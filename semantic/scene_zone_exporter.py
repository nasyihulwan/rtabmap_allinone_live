#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════╗
║  scene_zone_exporter.py — Scene-Zone Grid Exporter (SegFormer)  ║
║                                                                  ║
║  STATUS: skeleton, divalidasi via replay bag + RTABMap localize ║
║  (lihat memory plan_scene_zone_grid.md, roadmap #4).             ║
║                                                                  ║
║  Beda dari semantic_map_exporter.py: bukan daftar objek titik,  ║
║  melainkan grid sel berlabel (scene zone) — tiap sel = label    ║
║  kelas navigasi SegFormer paling sering muncul di situ          ║
║  (majority vote), bukan occupancy.                              ║
║                                                                  ║
║  Live node — jalankan BERSAMA (bukan sendirian):                ║
║    ros2 bag play <bag> --clock                                  ║
║    RTABMap (localization:=true, db_path=rtabmap.db)              ║
║    segformer_node.py                                            ║
║  supaya frame "map" tersedia (RTABMap yang publish map→odom     ║
║  secara live — tidak ada di /tf mentah bag, sudah dibuktikan   ║
║  di percobaan offline sebelumnya).                               ║
║                                                                  ║
║  Subscribe:                                                      ║
║    /semantic/segformer/mask_full   (mono8, ADE20K class id)     ║
║    /camera/camera/aligned_depth_to_color/image_raw (16UC1)      ║
║    /camera/camera/color/camera_info                              ║
║    TF map → camera_color_optical_frame                          ║
║                                                                  ║
║  Output (format diputuskan di riset 2026-07-06):                ║
║    <out>_labels.png   — 8-bit, 1 channel, label id kompak/sel   ║
║    <out>_labels.yaml  — resolution/origin/label_map sidecar     ║
║                                                                  ║
║  Grid di-align PERSIS ke lab_map.pgm (width/height/resolution/  ║
║  origin sama) supaya index sel bisa langsung dipetakan 1:1.     ║
╚══════════════════════════════════════════════════════════════════╝
"""

import os
import threading
from collections import Counter, deque
from scipy import ndimage

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
import tf2_ros
from cv_bridge import CvBridge

import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from segformer_node import SegformerNode  # noqa: E402 — reuse _run_inference (mode offline-db)


class _HeadlessInference:
    """Reuse SegformerNode._run_inference tanpa init rclpy Node penuh (mode offline-db)."""

    def __init__(self, model_path: str, device: str = 'cuda', input_size: int = 512):
        import torch
        from transformers import SegformerImageProcessor, SegformerForSemanticSegmentation
        self.device = torch.device(device if torch.cuda.is_available() else 'cpu')
        self.input_size = input_size
        self.processor = SegformerImageProcessor.from_pretrained(model_path)
        self.model = SegformerForSemanticSegmentation.from_pretrained(model_path).to(self.device).eval()

    def run(self, bgr: np.ndarray) -> np.ndarray:
        return SegformerNode._run_inference(self, bgr)

# ── Label kompak: ADE20K class id → id kecil 1-8 (0 = no_data) ─────────────
# Sinkron dengan KEEP_CLASSES di segformer_node.py
ADE20K_TO_COMPACT = {
    0:  1,  # wall
    3:  2,  # floor
    5:  3,  # ceiling
    6:  4,  # corridor
    29: 5,  # stairs
    52: 6,  # door
    53: 7,  # window
    55: 8,  # railing
}
COMPACT_LABEL_NAMES = {
    0: 'no_data', 1: 'wall', 2: 'floor', 3: 'ceiling', 4: 'corridor',
    5: 'stairs', 6: 'door', 7: 'window', 8: 'railing',
}

MAP_FRAME = 'map'
CAMERA_OPTICAL_FRAME = 'camera_color_optical_frame'


def _quat_to_rotation_matrix(x, y, z, w) -> np.ndarray:
    """Sama seperti semantic_map_exporter.py — no scipy dependency."""
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


class SceneZoneGrid:
    """Akumulator per-sel: Counter label per (row, col), align ke lab_map.pgm."""

    def __init__(self, map_yaml_path: str):
        with open(map_yaml_path) as f:
            m = yaml.safe_load(f)
        self.resolution = float(m['resolution'])
        self.origin_x, self.origin_y = float(m['origin'][0]), float(m['origin'][1])

        from PIL import Image as PILImage
        ref = PILImage.open(os.path.join(os.path.dirname(map_yaml_path), m['image']))
        self.width, self.height = ref.size  # (791, 472) pada lab_map saat ini

        # counts[(row_from_bottom, col)] = Counter({compact_label_id: n_observasi})
        self.counts: dict = {}

    def world_to_cell(self, wx: float, wy: float):
        col = int((wx - self.origin_x) / self.resolution)
        row = int((wy - self.origin_y) / self.resolution)
        if 0 <= col < self.width and 0 <= row < self.height:
            return row, col
        return None

    def add_observation(self, wx: float, wy: float, compact_label: int):
        cell = self.world_to_cell(wx, wy)
        if cell is None or compact_label == 0:
            return
        self.counts.setdefault(cell, Counter())[compact_label] += 1

    def majority_vote_array(self) -> np.ndarray:
        """-> uint8 array (height, width), row 0 = ATAS gambar (konvensi PGM)."""
        grid = np.zeros((self.height, self.width), dtype=np.uint8)
        for (row, col), counter in self.counts.items():
            label, _ = counter.most_common(1)[0]
            img_row = self.height - 1 - row
            grid[img_row, col] = label
        return grid

    def write(self, out_prefix: str, override_array: np.ndarray = None):
        grid = self.majority_vote_array() if override_array is None else override_array
        from PIL import Image as PILImage
        os.makedirs(os.path.dirname(out_prefix) or '.', exist_ok=True)
        PILImage.fromarray(grid, mode='L').save(f'{out_prefix}_labels.png')

        sidecar = {
            'image':      f'{os.path.basename(out_prefix)}_labels.png',
            'resolution': self.resolution,
            'origin':     [self.origin_x, self.origin_y, 0],
            'width':      self.width,
            'height':     self.height,
            'label_map':  COMPACT_LABEL_NAMES,
        }
        with open(f'{out_prefix}_labels.yaml', 'w') as f:
            yaml.dump(sidecar, f, default_flow_style=False)

        n_cells = int((grid > 0).sum())
        return n_cells


class SceneZoneExporter(Node):
    def __init__(self):
        super().__init__('scene_zone_exporter')

        self.declare_parameter('map_yaml_path', '/mnt/ssd/Nasyih_MBKM/maps/lab_map.yaml')
        self.declare_parameter('out_prefix',     '/tmp/scene_zone')
        self.declare_parameter('export_rate',    5.0)
        self.declare_parameter('sample_step',    4)
        self.declare_parameter('min_depth',      0.2)
        self.declare_parameter('max_depth',      8.0)
        self.declare_parameter('sync_slop',      0.5)

        map_yaml   = self.get_parameter('map_yaml_path').value
        self.out_prefix = self.get_parameter('out_prefix').value
        export_rate = self.get_parameter('export_rate').value
        self.step   = self.get_parameter('sample_step').value
        self.min_d  = self.get_parameter('min_depth').value
        self.max_d  = self.get_parameter('max_depth').value
        slop        = self.get_parameter('sync_slop').value

        self.grid = SceneZoneGrid(map_yaml)
        self._lock = threading.Lock()

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.fx = self.fy = self.cx = self.cy = None

        qos_be = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        qos_rel = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info',
            self._cam_info_cb, qos_rel)

        sub_mask = message_filters.Subscriber(
            self, Image, '/semantic/segformer/mask_full', qos_profile=qos_rel)
        sub_dep = message_filters.Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw', qos_profile=qos_be)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [sub_mask, sub_dep], queue_size=10, slop=slop)
        self.sync.registerCallback(self._sync_cb)

        self.create_timer(export_rate, self._export)

        self._frames_processed = 0
        self._points_added = 0
        self.get_logger().info(
            f'SceneZoneExporter ready | grid {self.grid.width}x{self.grid.height} '
            f'@ {self.grid.resolution}m | out={self.out_prefix}')

    def _cam_info_cb(self, msg: CameraInfo):
        if self.fx is not None:
            return
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]
        self.get_logger().info(
            f'Intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} cx={self.cx:.1f} cy={self.cy:.1f}')

    def _sync_cb(self, mask_msg: Image, dep_msg: Image):
        if self.fx is None:
            return

        mask = np.frombuffer(mask_msg.data, dtype=np.uint8).reshape(
            mask_msg.height, mask_msg.width)
        dep = np.frombuffer(dep_msg.data, dtype=np.uint16).reshape(
            dep_msg.height, dep_msg.width).astype(np.float32) / 1000.0

        try:
            t = self.tf_buffer.lookup_transform(
                MAP_FRAME, CAMERA_OPTICAL_FRAME, mask_msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=4.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'[TF] lookup gagal: {e}', throttle_duration_sec=5.0)
            return

        tr, ro = t.transform.translation, t.transform.rotation
        R = _quat_to_rotation_matrix(ro.x, ro.y, ro.z, ro.w)
        T = np.array([tr.x, tr.y, tr.z])

        h, w = mask.shape
        n_added = 0
        with self._lock:
            for v in range(0, h, self.step):
                for u in range(0, w, self.step):
                    compact = ADE20K_TO_COMPACT.get(int(mask[v, u]), 0)
                    if compact == 0:
                        continue
                    Z = float(dep[v, u])
                    if Z < self.min_d or Z > self.max_d:
                        continue
                    X = (u - self.cx) * Z / self.fx
                    Y = (v - self.cy) * Z / self.fy
                    world = R @ np.array([X, Y, Z]) + T
                    self.grid.add_observation(float(world[0]), float(world[1]), compact)
                    n_added += 1

        self._frames_processed += 1
        self._points_added += n_added
        if self._frames_processed % 20 == 1:
            self.get_logger().info(
                f'[Progress] frames={self._frames_processed} '
                f'points_added={self._points_added} cells={len(self.grid.counts)}')

    def _export(self):
        with self._lock:
            n_cells = self.grid.write(self.out_prefix)
        self.get_logger().info(
            f'[Export] {n_cells} sel berlabel -> {self.out_prefix}_labels.png/.yaml')


# ═════════════════════════════════════════════════════════════════════
# OFFLINE MODE — pakai pose keyframe yg SUDAH ADA di rtabmap.db
# (read-only, TIDAK menjalankan RTABMap live sama sekali)
#
# Kenapa: replay live RTABMap localization mode ternyata tidak stabil
# (lihat memory plan_scene_zone_grid.md GAP #2/#3 — loop closure sering
# ditolak/ditahan, map TF gak reliable). Tapi db ini SUDAH punya tabel
# Node berisi pose ter-optimasi utk 241 keyframe dari bag yg SAMA persis
# (stamp Node cocok dgn timestamp bag, translation dalam batas lab_map.pgm).
# Jadi "localize"-nya sudah beres sejak proses mapping awal — tinggal
# dipakai read-only, tanpa perlu re-localize live.
# ═════════════════════════════════════════════════════════════════════

def _read_node_poses(db_path: str):
    """Baca (stamp, 4x4 pose map<-baselink) dari rtabmap.db. READ-ONLY (mode=ro)."""
    import sqlite3
    conn = sqlite3.connect(f'file:{db_path}?mode=ro', uri=True)
    cur = conn.cursor()
    cur.execute('SELECT stamp, pose FROM Node ORDER BY id')
    out = []
    for stamp, blob in cur.fetchall():
        mat34 = np.frombuffer(blob, dtype=np.float32).reshape(3, 4)
        T = np.eye(4)
        T[:3, :] = mat34
        out.append((float(stamp), T))
    conn.close()
    return out


def _read_tf_buffer_static_only(bag_path: str):
    """Isi tf2_ros.Buffer dari /tf + /tf_static di bag — cukup utk static chain
    (base_link -> camera_color_optical_frame), TIDAK butuh frame 'map'."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage

    buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=400.0))
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/tf', '/tf_static']))
    while reader.has_next():
        topic, data, _t = reader.read_next()
        msg = deserialize_message(data, TFMessage)
        for transform in msg.transforms:
            if topic == '/tf_static':
                buffer.set_transform_static(transform, 'bag')
            else:
                buffer.set_transform(transform, 'bag')
    return buffer


def _rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """3x3 -> quaternion (x,y,z,w). Standard Shepperd's method."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 0.5 / np.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def _quat_slerp(q0: np.ndarray, q1: np.ndarray, alpha: float) -> np.ndarray:
    if np.dot(q0, q1) < 0:
        q1 = -q1
    dot = np.clip(np.dot(q0, q1), -1.0, 1.0)
    if dot > 0.9995:
        result = q0 + alpha * (q1 - q0)
        return result / np.linalg.norm(result)
    theta_0 = np.arccos(dot)
    theta = theta_0 * alpha
    q2 = q1 - q0 * dot
    q2 = q2 / np.linalg.norm(q2)
    return q0 * np.cos(theta) + q2 * np.sin(theta)


class _PoseInterpolator:
    """Interpolasi SE(3) (linear translation + slerp rotation) antar keyframe
    Node db, supaya dapat pose utk timestamp SEMBARANG (bukan cuma 241 keyframe)."""

    def __init__(self, nodes):
        nodes = sorted(nodes, key=lambda n: n[0])
        self.stamps = np.array([n[0] for n in nodes])
        self.trans = np.array([n[1][:3, 3] for n in nodes])
        self.quats = np.array([_rotation_matrix_to_quaternion(n[1][:3, :3]) for n in nodes])

    def pose_at(self, t: float):
        idx = np.searchsorted(self.stamps, t)
        if idx <= 0 or idx >= len(self.stamps):
            return None
        i0, i1 = idx - 1, idx
        span = self.stamps[i1] - self.stamps[i0]
        if span <= 0 or (t - self.stamps[i0]) > 2.0:  # jangan ekstrapolasi kalau gap keyframe > 2s
            return None
        alpha = (t - self.stamps[i0]) / span
        trans = self.trans[i0] + alpha * (self.trans[i1] - self.trans[i0])
        quat = _quat_slerp(self.quats[i0], self.quats[i1], alpha)
        R = _quat_to_rotation_matrix(*quat)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = trans
        return T


def _fill_zones_geodesic(zone_grid: np.ndarray, free_mask: np.ndarray) -> np.ndarray:
    """Sebarkan label zona (1-4) ke semua sel free-space kosong di sekitarnya,
    via multi-source BFS 4-connected — TIDAK menembus tembok (BFS cuma jalan
    di dalam free_mask). Sel kosong dapat label dari sumber TERDEKAT secara
    geodesic (jarak lewat lorong asli), bukan garis lurus — jadi gak leak ke
    ruangan sebelah kalau dipisah tembok, walau tembok tipis.
    """
    from collections import deque
    h, w = zone_grid.shape
    filled = zone_grid.copy()
    visited = (filled > 0)
    q = deque()
    for y, x in zip(*np.where(visited)):
        q.append((y, x))

    while q:
        y, x = q.popleft()
        lbl = filled[y, x]
        for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w and free_mask[ny, nx] and not visited[ny, nx]:
                visited[ny, nx] = True
                filled[ny, nx] = lbl
                q.append((ny, nx))
    return filled


def _smooth_label_grid(grid_array: np.ndarray, window: int = 3) -> np.ndarray:
    """Post-filter: tiap sel diganti label paling sering muncul di tetangga
    (window x window), abaikan no_data(0) di perhitungan tetangga. Sel yang
    sama sekali tidak punya observasi (grid==0 dan tetangga juga kosong)
    tetap no_data — tidak menghalusinasi label ke area yang tidak teramati."""
    import scipy.ndimage as ndi
    h, w = grid_array.shape
    n_labels = len(COMPACT_LABEL_NAMES) - 1  # 1..8
    counts = np.zeros((n_labels, h, w), dtype=np.float32)
    for lbl in range(1, n_labels + 1):
        counts[lbl - 1] = ndi.uniform_filter(
            (grid_array == lbl).astype(np.float32), size=window) * (window * window)
    best_idx = counts.argmax(axis=0)
    best_count = counts.max(axis=0)
    smoothed = np.where(best_count > 0, best_idx + 1, 0).astype(np.uint8)
    return smoothed


def run_offline_from_db(bag_path: str, db_path: str, map_yaml_path: str,
                         model_path: str, out_prefix: str,
                         base_frame: str = 'a200_1060/base_link',
                         every_n: int = 3, max_depth: float = 4.0,
                         smooth: bool = True):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo

    grid = SceneZoneGrid(map_yaml_path)
    nodes = _read_node_poses(db_path)
    print(f'[DB] {len(nodes)} keyframe pose dibaca (read-only) dari {db_path}')
    pose_interp = _PoseInterpolator(nodes)

    tf_buffer = _read_tf_buffer_static_only(bag_path)
    # Static transform base_link -> camera_color_optical_frame: konstan,
    # jadi timestamp query sembarang selama ada di dalam window bag.
    probe_stamp = Time(seconds=nodes[0][0]).to_msg()
    cam_tf = tf_buffer.lookup_transform(
        base_frame, CAMERA_OPTICAL_FRAME, probe_stamp,
        timeout=rclpy.duration.Duration(seconds=1.0))
    tr, ro = cam_tf.transform.translation, cam_tf.transform.rotation
    R_bc = _quat_to_rotation_matrix(ro.x, ro.y, ro.z, ro.w)
    T_bc = np.eye(4)
    T_bc[:3, :3] = R_bc
    T_bc[:3, 3] = [tr.x, tr.y, tr.z]
    print(f'[TF] base_link->camera static: t=({tr.x:.3f},{tr.y:.3f},{tr.z:.3f})')

    inference = _HeadlessInference(model_path)
    bridge = CvBridge()

    # SINGLE PASS streaming — TIDAK menyimpan seluruh bag di RAM (percobaan
    # pertama preload semua depth+color sekaligus, ~20GB RAM, nyaris OOM).
    # Depth cuma disimpan di ring-buffer kecil (mirip pola live-node),
    # karena color & depth datang interleaved dlm urutan waktu perekaman asli.
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[
        '/camera/camera/color/image_raw',
        '/camera/camera/aligned_depth_to_color/image_raw',
        '/camera/camera/color/camera_info',
    ]))

    fx = fy = cx = cy = None
    depth_ring: list = []  # [(stamp_ns, uint16 depth mm)] max ~10 entri
    RING_MAX = 10

    n_processed = 0
    n_skipped_no_pose = 0
    n_skipped_no_depth = 0
    total_points = 0
    color_idx = -1

    while reader.has_next():
        topic, data, t = reader.read_next()

        if topic == '/camera/camera/color/camera_info':
            if fx is None:
                msg = deserialize_message(data, CameraInfo)
                fx, fy, cx, cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
                print(f'[Intrinsics] fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}')
            continue

        if topic == '/camera/camera/aligned_depth_to_color/image_raw':
            msg = deserialize_message(data, Image)
            dep_mm = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            depth_ring.append((t, dep_mm))
            if len(depth_ring) > RING_MAX:
                depth_ring.pop(0)
            continue

        if topic != '/camera/camera/color/image_raw' or fx is None:
            continue

        color_idx += 1
        if color_idx % every_n != 0:
            continue

        stamp_s = t / 1e9
        T_map_base = pose_interp.pose_at(stamp_s)
        if T_map_base is None:
            n_skipped_no_pose += 1
            continue
        if not depth_ring:
            n_skipped_no_depth += 1
            continue

        depth_t, dep_mm = min(depth_ring, key=lambda item: abs(item[0] - t))
        if abs(depth_t - t) > int(0.1 * 1e9):
            n_skipped_no_depth += 1
            continue
        dep = dep_mm.astype(np.float32) / 1000.0

        color_msg = deserialize_message(data, Image)
        bgr = bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')

        mask_full = inference.run(bgr)

        T_map_cam = T_map_base @ T_bc
        R = T_map_cam[:3, :3]
        T = T_map_cam[:3, 3]

        h, w = mask_full.shape
        step = 4
        n_added = 0
        for v in range(0, h, step):
            for u in range(0, w, step):
                compact = ADE20K_TO_COMPACT.get(int(mask_full[v, u]), 0)
                if compact == 0:
                    continue
                Z = float(dep[v, u])
                if Z < 0.2 or Z > max_depth:
                    continue
                X = (u - cx) * Z / fx
                Y = (v - cy) * Z / fy
                world = R @ np.array([X, Y, Z]) + T
                grid.add_observation(float(world[0]), float(world[1]), compact)
                n_added += 1

        n_processed += 1
        total_points += n_added
        if n_processed % 100 == 1:
            print(f'[Progress] color_idx={color_idx} processed={n_processed} '
                  f'points_added_total={total_points} cells={len(grid.counts)}')

    print(f'[Info] {n_skipped_no_pose} frame dilewati (di luar rentang pose), '
          f'{n_skipped_no_depth} frame dilewati (depth gak sinkron)')

    smoothed = None
    if smooth:
        raw = grid.majority_vote_array()
        smoothed = _smooth_label_grid(raw, window=3)

    n_cells = grid.write(out_prefix, override_array=smoothed)
    print(f'[Export] selesai: {n_processed} frame diproses (every_n={every_n}), '
          f'{n_cells} sel berlabel -> {out_prefix}_labels.png/.yaml')


# ═════════════════════════════════════════════════════════════════════
# MODE ZONA — pakai classify_scene() (label scene per-frame: corridor/
# lab_workshop/doorway/staircase/unknown), BUKAN kelas ADE20K mentah per-pixel.
# Geometri (wall/unknown/free) diambil LANGSUNG dari lab_map.pgm (lidar,
# sudah presisi) — grid kamera cuma dipakai buat mewarnai ZONA di dalam
# free-space yang sudah pasti benar, bukan buat nentuin bentuknya.
# ═════════════════════════════════════════════════════════════════════

SCENE_LABEL_TO_ZONE = {
    'unknown': 0, 'corridor': 1, 'lab_workshop': 2, 'doorway': 3, 'staircase': 4,
}
ZONE_NAMES = {0: 'unknown', 1: 'corridor', 2: 'lab_workshop', 3: 'doorway', 4: 'staircase', 5: 'wall'}
# id ADE20K yg dianggap "lantai yg bisa dipijak" -> zona frame ini dilukis di situ
GROUND_ADE20K_IDS = {3, 6}  # floor, corridor


def _load_semantic_objects(semantic_map_yaml: str, min_cell_count: int = 10000):
    """Baca semantic_map.yaml (hasil Fase B) -> list of (x, y, class_name).

    [fix] filter cell_count minimum: semantic_map.yaml bisa berisi instance
    duplikat/false-positive dari YOLO (mis. 7 instance "stairs" tersebar di
    sepanjang koridor, cuma 1 yg asli — dibuktikan cell_count 69701 vs
    69-5141 utk yg lain, ~60% trajectory kena radius kalau semua dipakai
    mentah-mentah). Instance dgn cell_count rendah = jarang keliatan =
    kemungkinan besar noise, bukan objek nyata. TIDAK mengubah
    semantic_map.yaml aslinya — cuma filter saat load di sini.
    """
    with open(semantic_map_yaml) as f:
        data = yaml.safe_load(f) or {}
    out = []
    n_skipped = 0
    for label, obj in data.items():
        if int(obj.get('cell_count', 0)) < min_cell_count:
            n_skipped += 1
            continue
        name = label.rsplit('_', 1)[0]
        out.append((float(obj['x']), float(obj['y']), name))
    print(f'[SemanticMap] {n_skipped} instance dilewati (cell_count < {min_cell_count})')
    return out


def _read_lab_map_geometry(map_yaml_path: str):
    """-> (occupied_mask, unknown_mask, free_mask) uint8 arrays, sesuai konvensi
    ROS map_server trinary (negate=0): 0=occupied(wall), 254=free, 205=unknown."""
    with open(map_yaml_path) as f:
        m = yaml.safe_load(f)
    from PIL import Image as PILImage
    arr = np.array(PILImage.open(os.path.join(os.path.dirname(map_yaml_path), m['image'])))
    occupied = arr < 50
    unknown = (arr > 180) & (arr < 230)
    free = arr > 230
    return occupied, unknown, free


def run_offline_zone_from_db(bag_path: str, db_path: str, map_yaml_path: str,
                              semantic_map_yaml: str, model_path: str, out_prefix: str,
                              base_frame: str = 'a200_1060/base_link',
                              every_n: int = 3, max_depth: float = 4.0,
                              object_radius: float = 4.0, min_cell_count: int = 10000):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo
    from segformer_node import classify_scene

    grid = SceneZoneGrid(map_yaml_path)
    nodes = _read_node_poses(db_path)
    pose_interp = _PoseInterpolator(nodes)
    print(f'[DB] {len(nodes)} keyframe pose dibaca (read-only) dari {db_path}')

    objects = _load_semantic_objects(semantic_map_yaml, min_cell_count=min_cell_count)
    obj_xy = np.array([[o[0], o[1]] for o in objects])
    obj_names = [o[2] for o in objects]
    print(f'[SemanticMap] {len(objects)} objek dibaca dari {semantic_map_yaml}')

    tf_buffer = _read_tf_buffer_static_only(bag_path)
    probe_stamp = Time(seconds=nodes[0][0]).to_msg()
    cam_tf = tf_buffer.lookup_transform(
        base_frame, CAMERA_OPTICAL_FRAME, probe_stamp,
        timeout=rclpy.duration.Duration(seconds=1.0))
    tr, ro = cam_tf.transform.translation, cam_tf.transform.rotation
    R_bc = _quat_to_rotation_matrix(ro.x, ro.y, ro.z, ro.w)
    T_bc = np.eye(4)
    T_bc[:3, :3] = R_bc
    T_bc[:3, 3] = [tr.x, tr.y, tr.z]

    inference = _HeadlessInference(model_path)
    bridge = CvBridge()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[
        '/camera/camera/color/image_raw',
        '/camera/camera/aligned_depth_to_color/image_raw',
        '/camera/camera/color/camera_info',
    ]))

    fx = fy = cx = cy = None
    depth_ring: list = []
    RING_MAX = 10
    n_processed = 0
    zone_hist = Counter()
    color_idx = -1

    while reader.has_next():
        topic, data, t = reader.read_next()

        if topic == '/camera/camera/color/camera_info':
            if fx is None:
                msg = deserialize_message(data, CameraInfo)
                fx, fy, cx, cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
                print(f'[Intrinsics] fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}')
            continue

        if topic == '/camera/camera/aligned_depth_to_color/image_raw':
            msg = deserialize_message(data, Image)
            dep_mm = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            depth_ring.append((t, dep_mm))
            if len(depth_ring) > RING_MAX:
                depth_ring.pop(0)
            continue

        if topic != '/camera/camera/color/image_raw' or fx is None:
            continue

        color_idx += 1
        if color_idx % every_n != 0:
            continue

        T_map_base = pose_interp.pose_at(t / 1e9)
        if T_map_base is None or not depth_ring:
            continue
        depth_t, dep_mm = min(depth_ring, key=lambda item: abs(item[0] - t))
        if abs(depth_t - t) > int(0.1 * 1e9):
            continue
        dep = dep_mm.astype(np.float32) / 1000.0

        color_msg = deserialize_message(data, Image)
        bgr = bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        mask_full = inference.run(bgr)

        # yolo_detections proxy: objek dari semantic_map.yaml dekat posisi robot
        robot_xy = T_map_base[:2, 3]
        if len(obj_xy) > 0:
            dists = np.linalg.norm(obj_xy - robot_xy, axis=1)
            yolo_detections = [obj_names[i] for i in np.where(dists < object_radius)[0]]
        else:
            yolo_detections = []

        scene_label = classify_scene(yolo_detections, mask_full, min_lab_objects=1)
        zone_id = SCENE_LABEL_TO_ZONE[scene_label]
        zone_hist[scene_label] += 1

        if zone_id == 0:  # unknown -> gak usah dilukis, biarkan grid apa adanya
            n_processed += 1
            continue

        T_map_cam = T_map_base @ T_bc
        R = T_map_cam[:3, :3]
        T = T_map_cam[:3, 3]

        h, w = mask_full.shape
        step = 4
        ground = np.isin(mask_full, list(GROUND_ADE20K_IDS))
        for v in range(0, h, step):
            for u in range(0, w, step):
                if not ground[v, u]:
                    continue
                Z = float(dep[v, u])
                if Z < 0.2 or Z > max_depth:
                    continue
                X = (u - cx) * Z / fx
                Y = (v - cy) * Z / fy
                world = R @ np.array([X, Y, Z]) + T
                grid.add_observation(float(world[0]), float(world[1]), zone_id)

        n_processed += 1
        if n_processed % 100 == 1:
            print(f'[Progress] color_idx={color_idx} processed={n_processed} '
                  f'cells={len(grid.counts)} zone_hist={dict(zone_hist)}')

    print(f'[Info] scene label histogram (per-frame): {dict(zone_hist)}')

    zone_grid = grid.majority_vote_array()  # nilai 0-4 (zona) per sel, 0=belum ada observasi
    zone_grid = _smooth_label_grid(zone_grid, window=3)

    occupied, unknown, free = _read_lab_map_geometry(map_yaml_path)
    zone_grid_filled = _fill_zones_geodesic(zone_grid, free)  # tutup "bolong" dlm 1 ruangan yg sama

    final = np.zeros_like(zone_grid)          # default 0 = unknown
    final[occupied] = 5                        # wall — langsung dari lab_map (lidar)
    final[free] = zone_grid_filled[free]       # zona kamera (sudah di-fill) HANYA berlaku di free-space

    os.makedirs(os.path.dirname(out_prefix) or '.', exist_ok=True)
    from PIL import Image as PILImage
    PILImage.fromarray(final.astype(np.uint8), mode='L').save(f'{out_prefix}_zone.png')
    sidecar = {
        'image': f'{os.path.basename(out_prefix)}_zone.png',
        'resolution': grid.resolution, 'origin': [grid.origin_x, grid.origin_y, 0],
        'width': grid.width, 'height': grid.height,
        'label_map': ZONE_NAMES,
    }
    with open(f'{out_prefix}_zone.yaml', 'w') as f:
        yaml.dump(sidecar, f, default_flow_style=False)
    print(f'[Export] selesai: {n_processed} frame diproses -> {out_prefix}_zone.png/.yaml')


# ═════════════════════════════════════════════════════════════════════
# MODE PURE-SEGFORMER + overlay YOLO — desain final (2026-07-06)
#
# Prinsip (per diskusi): SegFormer & YOLO punya tugas BEDA, jangan dicampur:
#   - SegFormer  -> "jenis AREA ini gimana bentuknya" (sempit dekat tembok
#                   = corridor, terbuka = open_area, ada tangga/pintu
#                   kelihatan) — PAKAI RASIO floor/wall/stair/door per
#                   frame, BUKAN per-pixel ADE20K mentah (v9 terbukti nyaris
#                   semua jadi "floor" doang, gak berguna) DAN BUKAN
#                   ngitung objek YOLO (itu bikin corridor ke-anggap lab
#                   cuma krn ada shelf, dan false-positive "stairs").
#   - YOLO (via semantic_map.yaml) -> TIDAK ikut nentuin sempit/lebar.
#                   Cuma nempel LABEL TAMBAHAN "workshop" di atas sel yg
#                   SUDAH ditentukan "open_area" oleh SegFormer, kalau
#                   dekat objek lab. Layer terpisah, saling melengkapi.
# ═════════════════════════════════════════════════════════════════════

PURE_ZONE_NAMES = {
    0: 'unknown', 1: 'corridor', 2: 'open_area', 3: 'doorway', 4: 'staircase',
    5: 'wall', 6: 'workshop',
}


def _classify_scene_pure_segformer(seg_mask: np.ndarray) -> str:
    """Sama seperti classify_scene() TAPI TANPA YOLO sama sekali — murni
    rasio kelas ADE20K per-frame. 'corridor' = sempit (wall_ratio tinggi),
    'open_area' = lebar (floor luas, wall_ratio rendah)."""
    seg_counts = Counter(seg_mask.flatten().tolist())
    total = seg_mask.size
    floor_ratio = seg_counts.get(3, 0) / total
    wall_ratio  = seg_counts.get(0, 0) / total
    stair_ratio = seg_counts.get(29, 0) / total
    door_ratio  = seg_counts.get(52, 0) / total

    if stair_ratio > 0.05:
        return 'staircase'
    elif door_ratio > 0.05:
        return 'doorway'
    elif floor_ratio > 0.3 and wall_ratio > 0.2:
        return 'corridor'
    elif floor_ratio > 0.3:
        return 'open_area'
    else:
        return 'unknown'


PURE_SCENE_TO_ID = {'unknown': 0, 'corridor': 1, 'open_area': 2, 'doorway': 3, 'staircase': 4}


def run_offline_purescene_from_db(bag_path: str, db_path: str, map_yaml_path: str,
                                   semantic_map_yaml: str, model_path: str, out_prefix: str,
                                   base_frame: str = 'a200_1060/base_link',
                                   every_n: int = 3, max_depth: float = 4.0,
                                   object_radius: float = 4.0, min_cell_count: int = 10000):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CameraInfo

    grid = SceneZoneGrid(map_yaml_path)
    nodes = _read_node_poses(db_path)
    pose_interp = _PoseInterpolator(nodes)
    print(f'[DB] {len(nodes)} keyframe pose dibaca (read-only) dari {db_path}')

    tf_buffer = _read_tf_buffer_static_only(bag_path)
    from rclpy.time import Time
    probe_stamp = Time(seconds=nodes[0][0]).to_msg()
    cam_tf = tf_buffer.lookup_transform(
        base_frame, CAMERA_OPTICAL_FRAME, probe_stamp,
        timeout=rclpy.duration.Duration(seconds=1.0))
    tr, ro = cam_tf.transform.translation, cam_tf.transform.rotation
    R_bc = _quat_to_rotation_matrix(ro.x, ro.y, ro.z, ro.w)
    T_bc = np.eye(4)
    T_bc[:3, :3] = R_bc
    T_bc[:3, 3] = [tr.x, tr.y, tr.z]

    inference = _HeadlessInference(model_path)
    bridge = CvBridge()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[
        '/camera/camera/color/image_raw',
        '/camera/camera/aligned_depth_to_color/image_raw',
        '/camera/camera/color/camera_info',
    ]))

    fx = fy = cx = cy = None
    depth_ring: list = []
    RING_MAX = 10
    n_processed = 0
    scene_hist = Counter()
    color_idx = -1

    while reader.has_next():
        topic, data, t = reader.read_next()

        if topic == '/camera/camera/color/camera_info':
            if fx is None:
                msg = deserialize_message(data, CameraInfo)
                fx, fy, cx, cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
                print(f'[Intrinsics] fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}')
            continue

        if topic == '/camera/camera/aligned_depth_to_color/image_raw':
            msg = deserialize_message(data, Image)
            dep_mm = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            depth_ring.append((t, dep_mm))
            if len(depth_ring) > RING_MAX:
                depth_ring.pop(0)
            continue

        if topic != '/camera/camera/color/image_raw' or fx is None:
            continue

        color_idx += 1
        if color_idx % every_n != 0:
            continue

        T_map_base = pose_interp.pose_at(t / 1e9)
        if T_map_base is None or not depth_ring:
            continue
        depth_t, dep_mm = min(depth_ring, key=lambda item: abs(item[0] - t))
        if abs(depth_t - t) > int(0.1 * 1e9):
            continue
        dep = dep_mm.astype(np.float32) / 1000.0

        color_msg = deserialize_message(data, Image)
        bgr = bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        mask_full = inference.run(bgr)

        scene_label = _classify_scene_pure_segformer(mask_full)  # TANPA YOLO
        zone_id = PURE_SCENE_TO_ID[scene_label]
        scene_hist[scene_label] += 1

        if zone_id == 0:
            n_processed += 1
            continue

        T_map_cam = T_map_base @ T_bc
        R = T_map_cam[:3, :3]
        T = T_map_cam[:3, 3]

        h, w = mask_full.shape
        step = 4
        ground = np.isin(mask_full, list(GROUND_ADE20K_IDS))
        for v in range(0, h, step):
            for u in range(0, w, step):
                if not ground[v, u]:
                    continue
                Z = float(dep[v, u])
                if Z < 0.2 or Z > max_depth:
                    continue
                X = (u - cx) * Z / fx
                Y = (v - cy) * Z / fy
                world = R @ np.array([X, Y, Z]) + T
                grid.add_observation(float(world[0]), float(world[1]), zone_id)

        n_processed += 1
        if n_processed % 100 == 1:
            print(f'[Progress] color_idx={color_idx} processed={n_processed} '
                  f'cells={len(grid.counts)} scene_hist={dict(scene_hist)}')

    print(f'[Info] scene histogram (pure SegFormer, no YOLO): {dict(scene_hist)}')

    zone_grid = grid.majority_vote_array()
    zone_grid = _smooth_label_grid(zone_grid, window=5)

    occupied, unknown, free = _read_lab_map_geometry(map_yaml_path)
    zone_grid_filled = _fill_zones_geodesic(zone_grid, free)

    final = np.zeros_like(zone_grid)
    final[occupied] = 5
    final[free] = zone_grid_filled[free]

    # ── Overlay YOLO: tempel label "workshop" di atas open_area(2) yg dekat
    #    objek lab — TIDAK mengubah keputusan sempit/lebar, cuma nge-tag. ──
    objects = _load_semantic_objects(semantic_map_yaml, min_cell_count=min_cell_count)
    from segformer_node import LAB_OBJECTS
    lab_obj_xy = np.array([[x, y] for x, y, name in objects if name in LAB_OBJECTS])
    if len(lab_obj_xy) > 0:
        open_rows, open_cols = np.where(final == 2)
        for r_, c_ in zip(open_rows, open_cols):
            wx = grid.origin_x + (c_ + 0.5) * grid.resolution
            wy = grid.origin_y + (grid.height - 1 - r_ + 0.5) * grid.resolution
            if np.any(np.linalg.norm(lab_obj_xy - np.array([wx, wy]), axis=1) < object_radius):
                final[r_, c_] = 6  # workshop
    print(f'[Overlay] {len(lab_obj_xy)} objek lab dipakai utk tag workshop')

    os.makedirs(os.path.dirname(out_prefix) or '.', exist_ok=True)
    from PIL import Image as PILImage
    PILImage.fromarray(final.astype(np.uint8), mode='L').save(f'{out_prefix}_purescene.png')
    sidecar = {
        'image': f'{os.path.basename(out_prefix)}_purescene.png',
        'resolution': grid.resolution, 'origin': [grid.origin_x, grid.origin_y, 0],
        'width': grid.width, 'height': grid.height,
        'label_map': PURE_ZONE_NAMES,
    }
    with open(f'{out_prefix}_purescene.yaml', 'w') as f:
        yaml.dump(sidecar, f, default_flow_style=False)
    print(f'[Export] selesai: {n_processed} frame diproses -> {out_prefix}_purescene.png/.yaml')


# ═════════════════════════════════════════════════════════════════════
# MODE FINAL — segmentasi ruangan GEOMETRIS + overlay YOLO
# (desain final 2026-07-06, lihat plan_scene_zone_grid.md utk histori)
#
# TIDAK butuh bag/GPU sama sekali — cuma lab_map.pgm + semantic_map.yaml.
# Prinsip: corridor/open_area dari BENTUK PETA (erosion, motong lorong
# sempit dari ruangan lebar), BUKAN dari rasio SegFormer per-frame (v10
# terbukti gagal — wall_ratio>0.2 hampir selalu true, semua kebaca
# "corridor"). YOLO cuma nempel tag ekstra (workshop/staircase/doorway)
# di ruangan yg SUDAH dipisah scr geometris — bukan yg nentuin bentuknya.
# ═════════════════════════════════════════════════════════════════════

def _auto_erosion_radius_px(free: np.ndarray, occupied: np.ndarray,
                             percentile: float = 60.0) -> int:
    """Turunkan radius erosi OTOMATIS dari lebar koridor tipikal di peta INI
    SENDIRI — bukan angka piksel tetap — supaya fungsi ini tetap kepakai
    dgn benar di peta lain yg skalanya beda (koridor lebih lebar/sempit,
    resolusi grid beda) tanpa perlu tuning ulang manual tiap ganti lokasi.

    Cara: distance-transform tiap sel free ke tembok terdekat. Corridor
    (paling umum luasnya) mendominasi percentile bawah-menengah; ruangan
    lebar mendominasi percentile atas. Radius di-set sedikit di atas radius
    corridor tipikal (percentile default 60) — lorong seukuran corridor
    kepotong penuh saat erosi, ruangan yg jauh lebih lebar tetap utuh.
    """
    dist = ndimage.distance_transform_edt(~occupied)
    r = int(np.percentile(dist[free], percentile))
    return max(r, 3)


def _segment_rooms_by_erosion(free: np.ndarray, erosion_radius_px: int = 12):
    """Erosi free-space (motong lorong sempit) -> label komponen -> grow-back
    penuh via BFS geodesic. Return (room_id_grid, backbone_id)."""
    r = erosion_radius_px
    yy, xx = np.ogrid[-r:r+1, -r:r+1]
    disk = (xx**2 + yy**2) <= r**2
    eroded = ndimage.binary_erosion(free, structure=disk)
    seed_labels, n = ndimage.label(eroded, structure=np.ones((3, 3)))
    sizes = ndimage.sum(eroded, seed_labels, range(1, n+1))
    backbone_id = int(np.argmax(sizes)) + 1

    room_id_grid = np.zeros(free.shape, dtype=np.int32)
    room_id_grid[eroded] = seed_labels[eroded]
    visited = eroded.copy()
    q = deque(zip(*np.where(eroded)))
    h, w = free.shape
    while q:
        y, x = q.popleft()
        rid = room_id_grid[y, x]
        for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w and free[ny, nx] and not visited[ny, nx]:
                visited[ny, nx] = True
                room_id_grid[ny, nx] = rid
                q.append((ny, nx))
    return room_id_grid, backbone_id


def _nearest_labeled_room(room_id_grid, free, ry, rx, max_r=80):
    """Cari sel dgn room_id != 0 TERDEKAT dari (ry,rx) — skip sel free tanpa
    label (sisa BFS yg gak ke-cover). Dipakai krn posisi objek YOLO selalu
    jatuh di piksel OCCUPIED (objek itu sendiri = obstacle), jadi harus cari
    ruangan free-space terdekat di sekitarnya, bukan di titik itu sendiri."""
    h, w = free.shape
    labeled_mask = free & (room_id_grid != 0)
    for rad in range(4, max_r, 4):
        y0, y1 = max(0, ry-rad), min(h, ry+rad+1)
        x0, x1 = max(0, rx-rad), min(w, rx+rad+1)
        sub = labeled_mask[y0:y1, x0:x1]
        if sub.any():
            sub_room = room_id_grid[y0:y1, x0:x1]
            ys, xs = np.where(sub)
            dists = (ys+y0-ry)**2 + (xs+x0-rx)**2
            best = np.argmin(dists)
            return int(sub_room[ys[best], xs[best]])
    return 0


FINAL_ZONE_NAMES = {
    0: 'unknown', 1: 'corridor', 2: 'open_area', 3: 'doorway', 4: 'staircase',
    5: 'wall', 6: 'workshop',
}


def _merge_nearby_points(points, radius: float = 2.0):
    """points: [(x,y,cell_count), ...]. Gabung yg jaraknya < radius (meter) —
    YOLO sering deteksi objek fisik yg SAMA sbg instance terpisah (lihat kasus
    stairs 7x, door dobel). Per cluster, ambil yg cell_count tertinggi sbg wakil."""
    pts = sorted(points, key=lambda p: -p[2])
    used = [False] * len(pts)
    merged = []
    for i, (x, y, cc) in enumerate(pts):
        if used[i]:
            continue
        used[i] = True
        for j in range(i + 1, len(pts)):
            if used[j]:
                continue
            x2, y2, _ = pts[j]
            if (x - x2) ** 2 + (y - y2) ** 2 < radius ** 2:
                used[j] = True
        merged.append((x, y, cc))
    return merged


def _filter_doorways_by_furniture(doorway_mask: np.ndarray, grid: 'SceneZoneGrid',
                                   semantic_map_yaml: str, furniture_max_dist_m: float = 0.5):
    """Metode geometris (lebar sambungan) TIDAK BISA bedain "pintu asli antar
    ruangan" vs "celah sempit krn ada perabotan di tengah ruangan yg SAMA" —
    dua-duanya keliatan identik dari bentuk occupancy grid doang. Fix: pakai
    posisi objek YOLO (`semantic_map.yaml`) buat disambiguasi.

    [presisi] Cek jarak dari SETIAP PIKSEL segmen sempit (bukan cuma
    centroid-nya) ke objek terdekat, ambil yg PALING DEKAT. Kalau cuma pakai
    centroid, objek yg kebetulan ada di dekat pintu ASLI (mis. rak diletakkan
    di sebelah pintu, wajar di dunia nyata) ikut kehapus walau gak benar2
    menghalangi titik tersempitnya. Dgn cek per-piksel + threshold ketat
    (default 0.5m, bukan 1.0m), cuma kebuang kalau furniture-nya BENERAN
    persis di titik tersempit (menghalangi jalan), bukan sekadar di sekitar
    area pintu secara umum.
    """
    from scipy import ndimage as _ndi
    with open(semantic_map_yaml) as f:
        raw = yaml.safe_load(f) or {}
    objs = [(v['x'], v['y'], k.rsplit('_', 1)[0]) for k, v in raw.items()]
    obj_xy = np.array([[o[0], o[1]] for o in objs]) if objs else np.zeros((0, 2))
    obj_names = [o[2] for o in objs]
    furniture_idx = [i for i, name in enumerate(obj_names) if name not in ('door', 'stairs')]

    dlabeled, n = _ndi.label(doorway_mask, structure=np.ones((3, 3)))
    keep_mask = np.zeros_like(doorway_mask)
    for did in range(1, n + 1):
        ys, xs = np.where(dlabeled == did)
        wxs = grid.origin_x + (xs + 0.5) * grid.resolution
        wys = grid.origin_y + (grid.height - 1 - ys + 0.5) * grid.resolution

        min_furniture_dist = np.inf
        nearest_name = None
        if furniture_idx:
            for wx, wy in zip(wxs, wys):
                d = np.linalg.norm(obj_xy[furniture_idx] - np.array([wx, wy]), axis=1)
                i = int(np.argmin(d))
                if d[i] < min_furniture_dist:
                    min_furniture_dist = float(d[i])
                    nearest_name = obj_names[furniture_idx[i]]

        cy, cx = ys.mean(), xs.mean()
        wx_c = grid.origin_x + (cx + 0.5) * grid.resolution
        wy_c = grid.origin_y + (grid.height - 1 - cy + 0.5) * grid.resolution
        if min_furniture_dist < furniture_max_dist_m:
            print(f'[Doorway-filter] doorway @ ({wx_c:.2f},{wy_c:.2f}) dibuang — titik '
                  f'tersempitnya cuma {min_furniture_dist:.2f}m dari "{nearest_name}" (furniture)')
        else:
            keep_mask[dlabeled == did] = True
    return keep_mask.astype(bool)


class _UnionFind:
    def __init__(self):
        self.parent = {}

    def find(self, x):
        self.parent.setdefault(x, x)
        while self.parent[x] != x:
            self.parent[x] = self.parent[self.parent[x]]
            x = self.parent[x]
        return x

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.parent[ra] = rb


def _merge_fragments_and_find_doorways(room_id_grid: np.ndarray, free: np.ndarray,
                                        max_boundary_px: int, resolution: float = 0.05):
    """Erosi (`_segment_rooms_by_erosion`) sering memecah SATU ruangan fisik
    jadi beberapa fragmen kalau ada perabotan/sekat di dalamnya (fragmen2 itu
    nyambung lewat bukaan LEBAR satu sama lain, bukan pintu asli). Kalau
    dibiarkan: (a) fragmen yg jauh dari objek YOLO gak ke-tag apa2 (jadi
    'open_area' kosong padahal masih ruangan yg sama), (b) sambungan ANTAR
    fragmen itu malah ke-anggap doorway juga (over-detection).

    Fix: fragmen yg nyambung via bukaan LEBAR (> max_boundary_px) DIGABUNG
    jadi satu ruangan logis (union-find). Doorway CUMA ditandai di sambungan
    yg tetap SEMPIT **dan** antar ruangan yg SETELAH digabung masih beda
    (bukan ruangan yg sama, cuma kepecah fragmen erosi).

    User feedback (2026-07-06): begitu masuk ke suatu ruangan lewat 1 pintu
    dari koridor, SEMUA yg ada di dalam ruangan itu (walau kepecah2 fragmen
    krn perabotan, sambungannya sempit ATAU lebar) tetap SATU ruangan yg
    sama — closed space gak mungkin ada "sub-ruangan" lain di dalamnya.
    Jadi: sambungan ANTAR 2 fragmen yg SAMA2 BUKAN backbone SELALU digabung
    (gak peduli lebar sempitnya). Cuma sambungan YG LIBATKAN backbone yg
    dicek lebarnya (sempit=pintu asli tetap kepisah, lebar=gabung ke koridor).

    Return: (merged_room_id_grid, doorway_mask)
    """
    room_ids = sorted(set(room_id_grid[free].tolist()) - {0})
    uf = _UnionFind()
    for rid in room_ids:
        uf.find(rid)

    # backbone = fragmen TERBESAR SEBELUM digabung apapun (definisi koridor utama)
    pre_merge_ids, pre_merge_counts = np.unique(room_id_grid[free], return_counts=True)
    backbone_id = int(pre_merge_ids[np.argmax(pre_merge_counts)])

    # kumpulkan semua segmen batas dulu (lebar + lokasi), baru gabung
    segments = []  # (rid, nid, width_px, seg_mask)
    checked_pairs = set()
    for rid in room_ids:
        room_mask = (room_id_grid == rid) & free
        if room_mask.sum() < 20:
            continue
        dilated = ndimage.binary_dilation(room_mask, iterations=1)
        neighbor_ids = set(room_id_grid[dilated & free & (room_id_grid != rid) &
                                         (room_id_grid != 0)].tolist())
        for nid in neighbor_ids:
            pair = tuple(sorted((rid, nid)))
            if pair in checked_pairs:
                continue
            checked_pairs.add(pair)
            neighbor_mask = (room_id_grid == nid) & free
            boundary = dilated & neighbor_mask
            seg_labels, n_seg = ndimage.label(boundary, structure=np.ones((3, 3)))
            for seg_id in range(1, n_seg + 1):
                seg = (seg_labels == seg_id)
                segments.append((rid, nid, int(seg.sum()), seg))

    for rid, nid, width_px, seg in segments:
        involves_backbone = backbone_id in (rid, nid)
        if not involves_backbone:
            uf.union(rid, nid)  # sub-ruangan internal, gak peduli lebar -> selalu 1 ruangan
        elif width_px > max_boundary_px:
            uf.union(rid, nid)  # sambungan ke koridor yg LEBAR -> gabung ke koridor juga

    merged_grid = np.zeros_like(room_id_grid)
    for rid in room_ids:
        merged_grid[room_id_grid == rid] = uf.find(rid)
    backbone_root = uf.find(backbone_id)

    doorway_mask = np.zeros(free.shape, dtype=bool)
    for rid, nid, width_px, seg in segments:
        root_rid, root_nid = uf.find(rid), uf.find(nid)
        same_room_after_merge = root_rid == root_nid
        involves_backbone = backbone_root in (root_rid, root_nid)
        if (0 < width_px <= max_boundary_px and not same_room_after_merge and involves_backbone):
            doorway_mask |= ndimage.binary_dilation(seg, iterations=2) & free
            print(f'[Doorway-geo] room {rid}<->{nid}: {width_px}px '
                  f'(~{width_px*resolution:.2f}m) -> DITANDAI doorway')
        elif width_px > max_boundary_px:
            print(f'[Doorway-geo] room {rid}<->{nid}: {width_px}px '
                  f'(~{width_px*resolution:.2f}m) -> LEBAR, digabung jadi 1 ruangan')
        elif same_room_after_merge:
            print(f'[Doorway-geo] room {rid}<->{nid}: {width_px}px sempit TAPI sudah '
                  f'1 ruangan (sub-ruangan internal ATAU nyambung jg lewat bukaan lebar) -> skip')
    return merged_grid, doorway_mask


def run_final_geometry_yolo(map_yaml_path: str, semantic_map_yaml: str, out_prefix: str,
                             erosion_radius_m: float = None, min_cell_count: int = 10000,
                             door_min_cell_count: int = 100, door_merge_radius_m: float = 2.0,
                             doorway_max_width_m: float = 1.2):
    """Offline murni: lab_map.pgm (geometri) + semantic_map.yaml (posisi objek YOLO,
    read-only). TIDAK butuh bag/RTABMap/SegFormer live sama sekali.

    Semua parameter spasial dalam METER (bukan piksel mentah) supaya fungsi
    ini tetap benar dipakai di peta lain dgn resolusi/skala beda — piksel
    cuma detail internal, dikonversi pakai resolusi peta ybs saat runtime.
    `erosion_radius_m=None` -> dihitung OTOMATIS dari lebar koridor tipikal
    di peta ini sendiri (lihat `_auto_erosion_radius_px`), bukan angka tebakan.
    """
    from segformer_node import LAB_OBJECTS

    grid = SceneZoneGrid(map_yaml_path)
    occupied, unknown, free = _read_lab_map_geometry(map_yaml_path)
    h, w = free.shape
    res = grid.resolution

    if erosion_radius_m is None:
        erosion_radius_px = _auto_erosion_radius_px(free, occupied)
        print(f'[Geometry] erosion_radius auto-derived = {erosion_radius_px}px '
              f'(~{erosion_radius_px*res:.2f}m, dari persentil-60 lebar koridor peta ini)')
    else:
        erosion_radius_px = max(int(round(erosion_radius_m / res)), 1)
    doorway_max_width_px = max(int(round(doorway_max_width_m / res)), 1)

    room_id_grid, _ = _segment_rooms_by_erosion(free, erosion_radius_px)
    room_id_grid, geo_doorway = _merge_fragments_and_find_doorways(
        room_id_grid, free, doorway_max_width_px, resolution=res)
    # [nonaktif] filter furniture (_filter_doorways_by_furniture) TERBUKTI SALAH
    # buang pintu asli koridor<->workshop krn kebetulan ada sepatu/tempat
    # sampah di dekatnya (wajar di dunia nyata, bukan berarti menghalangi).
    # Dibiarkan tersedia sbg fungsi (belum dihapus) kalau mau dicoba lagi
    # dgn logika lebih presisi, tapi TIDAK dipanggil default lagi.
    merged_ids, counts = np.unique(room_id_grid[free], return_counts=True)
    backbone_id = int(merged_ids[np.argmax(counts)])
    print(f'[Geometry] backbone (corridor utama) = room_id {backbone_id}')

    def world_to_px(wx, wy):
        cell = grid.world_to_cell(wx, wy)
        if cell is None:
            return None
        row, col = cell
        return grid.height - 1 - row, col

    final = np.zeros(free.shape, dtype=np.uint8)
    final[free & (room_id_grid == backbone_id)] = 1                                # corridor
    final[free & (room_id_grid != backbone_id) & (room_id_grid != 0)] = 2          # open_area

    objects = _load_semantic_objects(semantic_map_yaml, min_cell_count=min_cell_count)
    stairs_xy = [(x, y) for x, y, name in objects if name == 'stairs']
    lab_xy    = [(x, y) for x, y, name in objects if name in LAB_OBJECTS]

    for wx, wy in stairs_xy:
        px = world_to_px(wx, wy)
        if px is None:
            continue
        rid = _nearest_labeled_room(room_id_grid, free, *px)
        if rid not in (0, backbone_id):
            final[free & (room_id_grid == rid)] = 4  # staircase
            print(f'[Tag] room_id={rid} -> staircase (objek stairs @ {wx:.2f},{wy:.2f})')

    for wx, wy in lab_xy:
        px = world_to_px(wx, wy)
        if px is None:
            continue
        rid = _nearest_labeled_room(room_id_grid, free, *px)
        if rid not in (0, backbone_id):
            m = free & (room_id_grid == rid) & (final != 4)
            if m.any():
                final[m] = 6  # workshop
                print(f'[Tag] room_id={rid} -> workshop (objek lab @ {wx:.2f},{wy:.2f})')

    # ── Doorway: GABUNGAN 2 sinyal ──────────────────────────────────────
    # 1) geometris (utama, sudah dihitung di atas bareng merge fragmen):
    #    sambungan SEMPIT antar ruangan YG BEDA (bukan dari YOLO sama
    #    sekali) — otomatis skip sambungan lebar/internal 1 ruangan.
    final[geo_doorway] = 3

    # 2) YOLO 'door' (sekunder, utk pintu yg mungkin gak ke-cover segmentasi
    #    ruangan, mis. pintu di sepanjang koridor sendiri). Instance dobel
    #    dari objek fisik yg sama (spt kasus stairs 7x) digabung dulu.
    with open(semantic_map_yaml) as f:
        _raw = yaml.safe_load(f) or {}
    raw_doors = [(v['x'], v['y'], v.get('cell_count', 0)) for k, v in _raw.items()
                 if k.rsplit('_', 1)[0] == 'door' and v.get('cell_count', 0) >= door_min_cell_count]
    merged_doors = _merge_nearby_points(raw_doors, radius=door_merge_radius_m)
    print(f'[Doorway-yolo] {len(raw_doors)} deteksi mentah -> {len(merged_doors)} setelah digabung')
    yy2, xx2 = np.ogrid[:h, :w]
    door_radius_px = max(int(round(0.3 / res)), 1)  # ~0.3m radius marker, skala ke resolusi peta
    for wx, wy, cc in merged_doors:
        px = world_to_px(wx, wy)
        if px is None:
            continue
        ry, rx = px
        disk2 = (yy2 - ry) ** 2 + (xx2 - rx) ** 2 <= door_radius_px ** 2
        final[free & disk2] = 3  # doorway

    result = np.zeros_like(final)
    result[occupied] = 5
    result[free] = final[free]

    os.makedirs(os.path.dirname(out_prefix) or '.', exist_ok=True)
    from PIL import Image as PILImage
    PILImage.fromarray(result.astype(np.uint8), mode='L').save(f'{out_prefix}_final.png')
    sidecar = {
        'image': f'{os.path.basename(out_prefix)}_final.png',
        'resolution': grid.resolution, 'origin': [grid.origin_x, grid.origin_y, 0],
        'width': grid.width, 'height': grid.height,
        'label_map': FINAL_ZONE_NAMES,
    }
    with open(f'{out_prefix}_final.yaml', 'w') as f:
        yaml.dump(sidecar, f, default_flow_style=False)

    names = FINAL_ZONE_NAMES
    counts = {names[l]: int((result == l).sum()) for l in np.unique(result)}
    print(f'[Export] cell counts: {counts}')
    print(f'[Export] selesai -> {out_prefix}_final.png/.yaml')


def main(args=None):
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=['live', 'offline-db', 'offline-db-zone',
                                        'offline-db-purescene', 'geometry-final'],
                     default='live')
    ap.add_argument('--erosion-radius-m', type=float, default=None,
                     help='default None = auto-derive dari lebar koridor tipikal peta ini')
    ap.add_argument('--door-min-cell-count', type=int, default=100)
    ap.add_argument('--door-merge-radius-m', type=float, default=2.0)
    ap.add_argument('--doorway-max-width-m', type=float, default=1.2)
    ap.add_argument('--bag')
    ap.add_argument('--db-path')
    ap.add_argument('--map-yaml', default='/mnt/ssd/Nasyih_MBKM/maps/lab_map.yaml')
    ap.add_argument('--semantic-map-yaml', default='/mnt/ssd/Nasyih_MBKM/home/.ros/semantic_map.yaml')
    ap.add_argument('--model', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'models', 'segformer_b0_ade'))
    ap.add_argument('--out', default='/tmp/scene_zone')
    ap.add_argument('--every-n', type=int, default=3, help='proses 1 dari N frame RGB')
    ap.add_argument('--max-depth', type=float, default=4.0)
    ap.add_argument('--object-radius', type=float, default=4.0)
    ap.add_argument('--min-cell-count', type=int, default=10000,
                     help='filter instance semantic_map.yaml dgn cell_count di bawah ini (noise)')
    ap.add_argument('--no-smooth', action='store_true', help='matikan post-filter smoothing')
    cli_args, ros_args = ap.parse_known_args(args=args)

    rclpy.init(args=ros_args)
    try:
        if cli_args.mode == 'offline-db':
            run_offline_from_db(cli_args.bag, cli_args.db_path, cli_args.map_yaml,
                                 cli_args.model, cli_args.out,
                                 every_n=cli_args.every_n, max_depth=cli_args.max_depth,
                                 smooth=not cli_args.no_smooth)
        elif cli_args.mode == 'offline-db-zone':
            run_offline_zone_from_db(cli_args.bag, cli_args.db_path, cli_args.map_yaml,
                                      cli_args.semantic_map_yaml, cli_args.model, cli_args.out,
                                      every_n=cli_args.every_n, max_depth=cli_args.max_depth,
                                      object_radius=cli_args.object_radius,
                                      min_cell_count=cli_args.min_cell_count)
        elif cli_args.mode == 'offline-db-purescene':
            run_offline_purescene_from_db(cli_args.bag, cli_args.db_path, cli_args.map_yaml,
                                           cli_args.semantic_map_yaml, cli_args.model, cli_args.out,
                                           every_n=cli_args.every_n, max_depth=cli_args.max_depth,
                                           object_radius=cli_args.object_radius,
                                           min_cell_count=cli_args.min_cell_count)
        elif cli_args.mode == 'geometry-final':
            run_final_geometry_yolo(cli_args.map_yaml, cli_args.semantic_map_yaml, cli_args.out,
                                     erosion_radius_m=cli_args.erosion_radius_m,
                                     min_cell_count=cli_args.min_cell_count,
                                     door_min_cell_count=cli_args.door_min_cell_count,
                                     door_merge_radius_m=cli_args.door_merge_radius_m,
                                     doorway_max_width_m=cli_args.doorway_max_width_m)
        else:
            node = SceneZoneExporter()
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                pass
            finally:
                node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
