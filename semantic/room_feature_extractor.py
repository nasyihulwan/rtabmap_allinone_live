#!/usr/bin/env python3
"""
room_feature_extractor.py — Per-ROOM SegFormer + YOLO feature aggregation.

Beda dari SceneZoneGrid (scene_zone_exporter.py) yang akumulasi PER SEL grid,
dan beda dari classify_scene() yang mutuskan PER FRAME: modul ini akumulasi
histogram kelas SegFormer PER ROOM (room_id dari RoomZoneMap — batas ruangan
sudah final dari geometry-final/lab_map.pgm, TIDAK diubah di sini) di
SELURUH trajectory. Noise per-frame (~85% konsistensi antar-frame, lihat
memory/plan_scene_zone_grid.md) diredam karena satu ruangan diobservasi
ribuan kali sepanjang bag, bukan cuma 1 frame.

Reuse (import, BUKAN reimplementasi):
  - scene_zone_exporter.py: _read_node_poses, _PoseInterpolator,
    _read_tf_buffer_static_only, _HeadlessInference, _quat_to_rotation_matrix,
    ADE20K_TO_COMPACT, COMPACT_LABEL_NAMES, _load_semantic_objects,
    CAMERA_OPTICAL_FRAME
  - room_zone_lookup.py: RoomZoneMap (batas ruangan, sudah tervalidasi)
  - segformer_node.py: classify_scene (utk baseline per-frame-majority, dipakai
    scene_zone_classifier.py sbg ablation vs classifier baru)

PENTING (testability): import berat (rclpy/torch/rosbag2_py/cv_bridge, dan
scene_zone_exporter.py itu sendiri yg meng-import semua itu di module-level)
DITUNDA ke dalam run_room_features_from_db() — supaya RoomHistogramAccumulator
(murni numpy, via RoomZoneMap yg juga murni numpy/scipy/PIL) tetap bisa
di-import & di-test di mesin tanpa ROS2/GPU sama sekali.

Output: <out> (YAML) — satu record per room_id:
  n_pixels, n_frames, ratio_<label> (8 kelas ADE20K compact),
  has_<yolo_object> (presence flag), classify_scene_majority (baseline label).

CLI:
  python3 room_feature_extractor.py --bag <bag> --db-path rtabmap.db \
      --zone-final-yaml scene_zone_final_v9_final.yaml \
      --semantic-map-yaml semantic_map.yaml --out room_features.yaml
"""

import os
import sys
from collections import Counter, defaultdict

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from room_zone_lookup import RoomZoneMap  # murni numpy/scipy/PIL/yaml, no ROS


class RoomHistogramAccumulator:
    """Akumulasi histogram kelas SegFormer PER ROOM (bukan per-sel/per-frame).

    Tiap observasi (world x,y, compact_label) di-snap ke room_id via
    RoomZoneMap.get_room_id_nearest_walkable(). Observasi yang tetap gak
    ke-resolve ke ruangan manapun (room_id==0 — di luar peta, atau literally
    di tembok/pintu/area belum terverifikasi) DIBUANG, bukan dipaksa ke
    ruangan terdekat — supaya histogram tiap ruangan hanya berisi observasi
    yang benar-benar dalam ruangan itu.
    """

    def __init__(self, room_zone_map: RoomZoneMap, n_classes: int = 8,
                 snap_radius_m: float = 0.3):
        self.room_zone_map = room_zone_map
        self.n_classes     = n_classes
        self.snap_radius_m = snap_radius_m

        # counts[room_id] = Counter({compact_label: n_observasi})
        self.counts: dict = defaultdict(Counter)
        # frame_counts[room_id] = jumlah frame yang nyumbang >=1 piksel ke ruangan ini
        self.frame_counts: Counter = Counter()

        self.n_added           = 0
        self.n_dropped_no_room = 0

    def add_observation(self, wx: float, wy: float, compact_label: int) -> int:
        """Tambah 1 observasi piksel. -> room_id yang di-update (0 kalau dibuang)."""
        if compact_label <= 0 or compact_label > self.n_classes:
            return 0
        room_id = self.room_zone_map.get_room_id_nearest_walkable(
            wx, wy, search_radius_m=self.snap_radius_m)
        if room_id <= 0:
            self.n_dropped_no_room += 1
            return 0
        self.counts[room_id][compact_label] += 1
        self.n_added += 1
        return room_id

    def mark_frame_rooms(self, room_ids_touched: set):
        """Panggil sekali per frame dgn set room_id yg disentuh frame itu."""
        for rid in room_ids_touched:
            self.frame_counts[rid] += 1

    def n_pixels(self, room_id: int) -> int:
        return sum(self.counts.get(room_id, {}).values())

    def ratios(self) -> dict:
        """-> {room_id: {compact_label(1..n): ratio}}, ratio dinormalisasi
        per-room (sum semua kelas = 1) — supaya lama-observasi tidak bocor
        jadi fitur (ruangan yg dilihat lebih lama tidak otomatis beda skala)."""
        out = {}
        for room_id, counter in self.counts.items():
            total = sum(counter.values())
            if total == 0:
                continue
            out[room_id] = {c: counter.get(c, 0) / total
                             for c in range(1, self.n_classes + 1)}
        return out


def _write_room_features(acc: RoomHistogramAccumulator, room_frame_labels: dict,
                          yolo_presence: dict, out_yaml: str, label_names: dict):
    """label_names: dict kompak {1..n: nama}, mis. ADE20K_TO_COMPACT punya
    lawan-mainnya COMPACT_LABEL_NAMES dari scene_zone_exporter.py."""
    ratios = acc.ratios()
    all_rooms = set(ratios) | set(room_frame_labels) | set(yolo_presence)

    records = {}
    for room_id in sorted(all_rooms):
        rec = {
            'n_pixels': acc.n_pixels(room_id),
            'n_frames': int(acc.frame_counts.get(room_id, 0)),
        }
        room_ratios = ratios.get(room_id, {})
        for c in range(1, acc.n_classes + 1):
            name = label_names.get(c, f'class{c}')
            rec[f'ratio_{name}'] = round(float(room_ratios.get(c, 0.0)), 6)

        for obj_name in sorted(yolo_presence.get(room_id, {})):
            rec[f'has_{obj_name}'] = True

        labels = room_frame_labels.get(room_id, [])
        if labels:
            rec['classify_scene_majority'] = Counter(labels).most_common(1)[0][0]

        records[int(room_id)] = rec

    os.makedirs(os.path.dirname(out_yaml) or '.', exist_ok=True)
    with open(out_yaml, 'w') as f:
        yaml.dump(records, f, default_flow_style=False, sort_keys=True)
    print(f'[Export] {len(records)} room -> {out_yaml}')
    return records


def run_room_features_from_db(bag_path: str, db_path: str, zone_final_yaml_path: str,
                               model_path: str, out_yaml: str,
                               semantic_map_yaml: str = None,
                               base_frame: str = 'a200_1060/base_link',
                               every_n: int = 3, max_depth: float = 4.0,
                               min_cell_count: int = 10000,
                               snap_radius_m: float = 0.3):
    """Replay bag + rtabmap.db (read-only, sama seperti run_offline_from_db di
    scene_zone_exporter.py), tapi akumulasi per ROOM (via RoomZoneMap) bukan
    per SEL grid. Dijalankan di mesin yg punya ROS2 + torch + bag/db (Orin)."""
    import rclpy
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rclpy.time import Time
    from sensor_msgs.msg import Image, CameraInfo
    from cv_bridge import CvBridge

    from scene_zone_exporter import (
        _read_node_poses, _PoseInterpolator, _read_tf_buffer_static_only,
        _HeadlessInference, _quat_to_rotation_matrix, ADE20K_TO_COMPACT,
        COMPACT_LABEL_NAMES, _load_semantic_objects, CAMERA_OPTICAL_FRAME,
    )
    from segformer_node import classify_scene

    room_map = RoomZoneMap(zone_final_yaml_path)
    print(f'[RoomMap] {room_map.n_rooms} room terdeteksi dari {zone_final_yaml_path}')
    n_classes = len(COMPACT_LABEL_NAMES) - 1  # -1 utk buang entry 'no_data'=0
    acc = RoomHistogramAccumulator(room_map, n_classes=n_classes, snap_radius_m=snap_radius_m)

    nodes = _read_node_poses(db_path)
    print(f'[DB] {len(nodes)} keyframe pose dibaca (read-only) dari {db_path}')
    pose_interp = _PoseInterpolator(nodes)

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
    print(f'[TF] base_link->camera static: t=({tr.x:.3f},{tr.y:.3f},{tr.z:.3f})')

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

    n_processed        = 0
    n_skipped_no_pose  = 0
    n_skipped_no_depth = 0
    color_idx          = -1
    room_frame_labels  = defaultdict(list)  # room_id -> [classify_scene per-frame label, ...]

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
        rooms_touched = set()
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
                room_id = acc.add_observation(float(world[0]), float(world[1]), compact)
                if room_id > 0:
                    rooms_touched.add(room_id)

        acc.mark_frame_rooms(rooms_touched)

        # Baseline (dipakai room_type_classifier.py sbg pembanding "naif"):
        # classify_scene() per-frame. Deteksi YOLO per-frame TIDAK tersedia di
        # replay offline ini (cuma ada semantic_map.yaml yg sudah teragregasi
        # dari sesi lain) — dipanggil dgn list kosong, sehingga jalur berbasis
        # objek YOLO di classify_scene() tidak pernah aktif dan hasilnya
        # murni dari rasio SegFormer per-frame (persis skenario yg didokumentasikan
        # gagal di memory/plan_scene_zone_grid.md — inilah baseline "naif"-nya).
        frame_label = classify_scene([], mask_full)
        for rid in rooms_touched:
            room_frame_labels[rid].append(frame_label)

        n_processed += 1
        if n_processed % 100 == 1:
            print(f'[Progress] color_idx={color_idx} processed={n_processed} '
                  f'rooms_seen={len(acc.counts)} pixels_added={acc.n_added} '
                  f'dropped_no_room={acc.n_dropped_no_room}')

    print(f'[Info] {n_skipped_no_pose} frame dilewati (di luar rentang pose), '
          f'{n_skipped_no_depth} frame dilewati (depth gak sinkron)')

    # ── YOLO presence features (pass ringan kedua, tanpa bag replay lagi) ──
    yolo_presence: dict = defaultdict(dict)
    if semantic_map_yaml and os.path.isfile(semantic_map_yaml):
        objects = _load_semantic_objects(semantic_map_yaml, min_cell_count=min_cell_count)
        n_dropped_obj = 0
        for x, y, name in objects:
            room_id = room_map.get_room_id_nearest_walkable(x, y, search_radius_m=snap_radius_m)
            if room_id <= 0:
                n_dropped_obj += 1
                continue
            yolo_presence[room_id][name] = True
        print(f'[YOLO] {len(objects)} objek dibaca, {n_dropped_obj} gak ke-resolve ke room manapun')
    else:
        print('[YOLO] semantic_map_yaml tidak diberikan/tidak ditemukan — skip fitur YOLO')

    return _write_room_features(acc, room_frame_labels, yolo_presence, out_yaml,
                                 COMPACT_LABEL_NAMES)


def main(args=None):
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True)
    ap.add_argument('--db-path', required=True)
    ap.add_argument('--zone-final-yaml', required=True,
                     help='mis. output/scene_zone_test/scene_zone_final_v9_final.yaml '
                          '(hasil --mode geometry-final di scene_zone_exporter.py)')
    ap.add_argument('--semantic-map-yaml', default=None)
    ap.add_argument('--model', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'models', 'segformer_b0_ade'))
    ap.add_argument('--out', default='/tmp/room_features.yaml')
    ap.add_argument('--every-n', type=int, default=3, help='proses 1 dari N frame RGB')
    ap.add_argument('--max-depth', type=float, default=4.0)
    ap.add_argument('--min-cell-count', type=int, default=10000,
                     help='filter instance semantic_map.yaml dgn cell_count di bawah ini (noise)')
    ap.add_argument('--snap-radius-m', type=float, default=0.3,
                     help='radius cari room walkable terdekat kalau piksel jatuh persis di '
                          'wall/doorway/unknown (lebih kecil dari default object_goal_nav '
                          'krn di sini snap piksel padat, bukan 1 goal point)')
    cli_args, ros_args = ap.parse_known_args(args=args)

    import rclpy
    rclpy.init(args=ros_args)
    try:
        run_room_features_from_db(
            cli_args.bag, cli_args.db_path, cli_args.zone_final_yaml, cli_args.model,
            cli_args.out, semantic_map_yaml=cli_args.semantic_map_yaml,
            every_n=cli_args.every_n, max_depth=cli_args.max_depth,
            min_cell_count=cli_args.min_cell_count, snap_radius_m=cli_args.snap_radius_m)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
