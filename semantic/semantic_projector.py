#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════╗
║  Semantic Projector — Segmentation + Instance Version           ║
║                                                                  ║
║  Subscribe:                                                      ║
║    /semantic/label_image        (mono8)  — class_id+1/pixel    ║
║    /semantic/instance_image     (mono8)  — instance_id/pixel   ║
║    /semantic/confidence_image   (mono8)  — round(conf*255)/pixel ║
║    /camera/camera/aligned_depth_to_color/image_raw (16UC1)  — depth               ║
║    /camera/color/camera_info    — intrinsics                    ║
║    /velodyne_points  (PointCloud2, opt-in via depth_source:=lidar)               ║
║                                                                  ║
║  Publish:                                                        ║
║    /semantic/pointcloud  (PointCloud2)                          ║
║      RGB encoding: R=class_id, G=instance_id, B=confidence*255 ║
║      → grid node decode class dan instance dari sini           ║
║                                                                  ║
║  Changelog:                                                      ║
║    [lidar] depth_source:=lidar — ganti sumber Z per-piksel dari  ║
║           depth stereo kamera ke range LiDAR (VLP-32C) yang     ║
║           diproyeksikan ke image plane, lebih stabil pada jarak ║
║           jauh dibanding depth stereo. Fallback ke depth kamera ║
║           di piksel yang tidak ter-cover titik LiDAR terdekat.  ║
║    [confidence] B channel (sebelumnya selalu 0) sekarang bawa   ║
║           confidence YOLO per piksel, dipakai position logger.  ║
╚══════════════════════════════════════════════════════════════════╝
"""

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import message_filters
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
import tf2_ros
import numpy as np
import struct

# [lidar] Sentinel "tidak ada data LiDAR" — SENGAJA bukan np.inf.
# cv2.erode meng-clip np.inf jadi float32-max (~3.4e38) secara diam-diam
# (dikonfirmasi lewat tes empiris), jadi np.isfinite() setelah erode tidak
# bisa dipakai untuk deteksi "belum ter-cover". Sentinel ini jauh di atas
# max_depth manapun yang masuk akal, dan dibandingkan pakai '<' saja.
NO_LIDAR = np.float32(1e4)


def _quat_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Sama persis dgn semantic_map_exporter.py — no scipy dependency."""
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def decode_pointcloud2_xyz(msg: PointCloud2):
    """
    Decode PointCloud2 → (N,3) float32 xyz array. Field-offset driven —
    TIDAK asumsikan point_step tertentu, supaya jalan baik untuk layout
    /semantic/pointcloud (16 byte) maupun driver Velodyne (22 byte:
    x,y,z,intensity float32 + ring uint16 + time float32).

    Returns (xyz, ok). xyz sudah dibuang baris NaN/Inf-nya (driver
    Velodyne kadang isi titik invalid dengan NaN untuk return kosong).
    """
    field_map = {f.name: f for f in msg.fields}
    if not all(k in field_map for k in ('x', 'y', 'z')):
        return None, False

    n_points = msg.width * msg.height
    if n_points == 0:
        return None, False

    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if len(raw) < n_points * msg.point_step:
        return None, False

    arr = raw.reshape(n_points, msg.point_step)
    ox, oy, oz = field_map['x'].offset, field_map['y'].offset, field_map['z'].offset
    x = arr[:, ox:ox+4].copy().view(np.float32).reshape(-1)
    y = arr[:, oy:oy+4].copy().view(np.float32).reshape(-1)
    z = arr[:, oz:oz+4].copy().view(np.float32).reshape(-1)

    xyz = np.stack([x, y, z], axis=1)
    finite = np.isfinite(xyz).all(axis=1)
    return xyz[finite], True


def rasterize_lidar_range(xyz_cam: np.ndarray, fx: float, fy: float,
                           cx: float, cy: float, h: int, w: int,
                           min_d: float, max_d: float, dilate_px: int):
    """
    Proyeksikan titik LiDAR (sudah di camera_color_optical_frame) ke image
    plane, hasilkan citra range (H,W) — nilai Z LiDAR menggantikan Z depth
    kamera di piksel yang ter-cover. NO_LIDAR di piksel tanpa titik terdekat.

    Kolisi piksel (>1 titik LiDAR jatuh di piksel sama): titik terdekat
    menang (scatter-min) — menghindari artefak "tembus" objek foreground.

    dilate_px: ukuran kernel min-filter (cv2.erode) untuk menyebarkan titik
    LiDAR yang jarang ke piksel tetangga kosong. 0 = tanpa dilasi (paling
    presisi, tapi coverage sangat jarang — VLP-32C jauh lebih jarang dari
    resolusi kamera).
    """
    X, Y, Z = xyz_cam[:, 0], xyz_cam[:, 1], xyz_cam[:, 2]
    valid = (Z > min_d) & (Z < max_d)
    if not valid.any():
        return None
    X, Y, Z = X[valid], Y[valid], Z[valid]

    u = np.round(fx * X / Z + cx).astype(np.int32)
    v = np.round(fy * Y / Z + cy).astype(np.int32)
    in_bounds = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    u, v, Z = u[in_bounds], v[in_bounds], Z[in_bounds]
    if len(Z) == 0:
        return None

    img = np.full((h, w), NO_LIDAR, dtype=np.float32)
    np.minimum.at(img, (v, u), Z)

    if dilate_px > 0:
        kernel = np.ones((dilate_px, dilate_px), np.uint8)
        img = cv2.erode(img, kernel)

    return img


class SemanticProjectorNode(Node):
    def __init__(self):
        super().__init__('semantic_projector')

        self.declare_parameter('max_depth',   8.0)
        self.declare_parameter('min_depth',   0.2)
        self.declare_parameter('sample_step', 4)
        self.declare_parameter('sync_slop',   0.5)
        # [lidar] 'camera' = perilaku lama (depth stereo saja, tidak berubah).
        # 'lidar' = fusi: Z per-piksel dari range LiDAR kalau ter-cover,
        # fallback ke depth kamera kalau tidak.
        self.declare_parameter('depth_source',   'camera')
        self.declare_parameter('lidar_topic',    '/velodyne_points')
        self.declare_parameter('lidar_dilate_px', 5)

        self.max_d = self.get_parameter('max_depth').value
        self.min_d = self.get_parameter('min_depth').value
        self.step  = self.get_parameter('sample_step').value
        slop       = self.get_parameter('sync_slop').value
        self.depth_source   = self.get_parameter('depth_source').value
        lidar_topic          = self.get_parameter('lidar_topic').value
        self.lidar_dilate_px = self.get_parameter('lidar_dilate_px').value

        if self.depth_source not in ('camera', 'lidar'):
            self.get_logger().warn(
                f"depth_source='{self.depth_source}' tidak dikenal, fallback ke 'camera'")
            self.depth_source = 'camera'

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

        sub_lbl  = message_filters.Subscriber(
            self, Image, '/semantic/label_image',    qos_profile=qos_be)
        sub_inst = message_filters.Subscriber(
            self, Image, '/semantic/instance_image', qos_profile=qos_be)
        sub_dep  = message_filters.Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw', qos_profile=qos_be)
        # [confidence] mono8, round(conf*255)/pixel — di-pack ke channel B
        # pointcloud (sebelumnya selalu 0, lihat _make_cloud).
        sub_conf = message_filters.Subscriber(
            self, Image, '/semantic/confidence_image', qos_profile=qos_be)

        if self.depth_source == 'lidar':
            # [lidar] TF buffer terpisah dari semantic_map_exporter.py —
            # node ini butuh velodyne→camera_color_optical_frame, exporter
            # butuh camera→map. Dua kebutuhan TF yang berbeda, dua Node ROS
            # yang berbeda proses, jadi masing-masing punya buffer sendiri.
            self.tf_buffer   = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

            sub_lidar = message_filters.Subscriber(
                self, PointCloud2, lidar_topic, qos_profile=qos_be)
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [sub_lbl, sub_inst, sub_dep, sub_conf, sub_lidar], queue_size=10, slop=slop)
            self.sync.registerCallback(self._sync_cb_lidar)
        else:
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [sub_lbl, sub_inst, sub_dep, sub_conf], queue_size=10, slop=slop)
            self.sync.registerCallback(self._sync_cb)

        self.pub = self.create_publisher(
            PointCloud2, '/semantic/pointcloud', 10)

        self._sc, self._tp = 0, 0
        self._lidar_hits, self._lidar_total = 0, 0
        self.get_logger().info(
            f'SemanticProjector ready | '
            f'depth: {self.min_d}-{self.max_d}m | step: {self.step} | '
            f'depth_source={self.depth_source}')

    def _cam_info_cb(self, msg):
        if self.fx is not None:
            return
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]
        self.get_logger().info(
            f'Intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} '
            f'cx={self.cx:.1f} cy={self.cy:.1f}')

    def _build_lidar_range_image(self, lidar_msg, target_stamp, h, w):
        """[lidar] TF velodyne→camera + proyeksi. None kalau TF/decode gagal."""
        xyz_lidar, ok = decode_pointcloud2_xyz(lidar_msg)
        if not ok or xyz_lidar is None or xyz_lidar.shape[0] == 0:
            return None
        try:
            t = self.tf_buffer.lookup_transform(
                'camera_color_optical_frame', lidar_msg.header.frame_id,
                target_stamp, timeout=rclpy.duration.Duration(seconds=0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'[lidar] TF {lidar_msg.header.frame_id}→camera_color_optical_frame gagal: {e}',
                throttle_duration_sec=5.0)
            return None

        tr, ro = t.transform.translation, t.transform.rotation
        R = _quat_to_rotation_matrix(ro.x, ro.y, ro.z, ro.w)
        T = np.array([tr.x, tr.y, tr.z], dtype=np.float64)
        xyz_cam = (R @ xyz_lidar.astype(np.float64).T).T + T

        return rasterize_lidar_range(
            xyz_cam, self.fx, self.fy, self.cx, self.cy, h, w,
            self.min_d, self.max_d, self.lidar_dilate_px)

    def _sync_cb(self, lbl_msg, inst_msg, dep_msg, conf_msg):
        self._process_frame(lbl_msg, inst_msg, dep_msg, conf_msg, lidar_range_img=None)

    def _sync_cb_lidar(self, lbl_msg, inst_msg, dep_msg, conf_msg, lidar_msg):
        lidar_range_img = self._build_lidar_range_image(
            lidar_msg, dep_msg.header.stamp,
            dep_msg.height, dep_msg.width) if self.fx is not None else None
        self._process_frame(lbl_msg, inst_msg, dep_msg, conf_msg, lidar_range_img)

    def _process_frame(self, lbl_msg, inst_msg, dep_msg, conf_msg, lidar_range_img):
        if self.fx is None:
            return

        lbl  = np.frombuffer(lbl_msg.data,  dtype=np.uint8).reshape(
            lbl_msg.height,  lbl_msg.width)
        inst = np.frombuffer(inst_msg.data, dtype=np.uint8).reshape(
            inst_msg.height, inst_msg.width)
        conf = np.frombuffer(conf_msg.data, dtype=np.uint8).reshape(
            conf_msg.height, conf_msg.width)
        dep  = np.frombuffer(dep_msg.data,  dtype=np.uint16).reshape(
            dep_msg.height,  dep_msg.width).astype(np.float32) / 1000.0

        if not (lbl > 0).any():
            return

        self._sc += 1
        h, w = lbl.shape
        points = []

        for v in range(0, h, self.step):
            for u in range(0, w, self.step):
                cls_val  = lbl[v, u]
                inst_val = inst[v, u]
                if cls_val == 0:
                    continue

                # [lidar] Pakai range LiDAR kalau piksel ini ter-cover,
                # else fallback ke depth kamera — satu jalur bounds-check
                # untuk kedua sumber supaya perlakuannya konsisten.
                Z_lidar = lidar_range_img[v, u] if lidar_range_img is not None else NO_LIDAR
                if lidar_range_img is not None and Z_lidar < NO_LIDAR:
                    Z = float(Z_lidar)
                    self._lidar_hits += 1
                else:
                    Z = float(dep[v, u])
                if lidar_range_img is not None:
                    self._lidar_total += 1
                if Z < self.min_d or Z > self.max_d:
                    continue

                X = (u - self.cx) * Z / self.fx
                Y = (v - self.cy) * Z / self.fy

                # Encode class + instance + confidence ke RGB
                # R = class_id (0-79)
                # G = instance_id (1-254)
                # B = round(confidence*255) — lihat /semantic/confidence_image
                r = int(cls_val) - 1   # class_id 0-indexed
                g = int(inst_val)
                b = int(conf[v, u])
                rgb_int = (r << 16) | (g << 8) | b
                rgb_f   = struct.unpack('f', struct.pack('I', rgb_int))[0]

                points.append((X, Y, Z, rgb_f))

        if not points:
            return

        self._tp += len(points)
        self.pub.publish(self._make_cloud(points, dep_msg.header))

        if self._sc % 10 == 1:
            msg = f'[Sync {self._sc}] Points: {len(points)} | Total: {self._tp}'
            if self.depth_source == 'lidar' and self._lidar_total > 0:
                hit_rate = self._lidar_hits / self._lidar_total
                msg += f' | lidar_hit_rate={hit_rate:.1%}'
            self.get_logger().info(msg)

    def _make_cloud(self, points, header):
        fields = [
            PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        data = bytearray()
        for (x, y, z, rgb) in points:
            data += struct.pack('ffff', x, y, z, rgb)

        msg = PointCloud2()
        msg.header = header
        # aligned_depth_to_color diproyeksikan dengan color camera intrinsics →
        # titik 3D ada di color optical frame, bukan depth optical frame
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.height = 1
        msg.width  = len(points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step   = 16 * len(points)
        msg.data       = bytes(data)
        msg.is_dense   = True
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = SemanticProjectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
