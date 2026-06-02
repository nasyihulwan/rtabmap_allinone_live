#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════╗
║  semantic_map_exporter.py — Semantic Map YAML Exporter          ║
║                                                                  ║
║  Subscribe:                                                      ║
║    /semantic/pointcloud  (PointCloud2, R=class_id, G=inst_id)  ║
║                                                                  ║
║  Accumulate posisi per instance → tulis ke:                     ║
║    ~/.ros/semantic_map.yaml                                      ║
║                                                                  ║
║  Format YAML:                                                    ║
║    chair_2:                                                      ║
║      x: 1.234                                                    ║
║      y: 5.678                                                    ║
║      class_id: 0                                                 ║
║      cell_count: 42                                              ║
╚══════════════════════════════════════════════════════════════════╝
"""

import os
import yaml
import struct
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2

SEMANTIC_MAP_PATH = os.path.expanduser('~/.ros/semantic_map.yaml')


def _load_class_names(model_path: str) -> dict:
    """Try to load class names from YOLO model. Returns {} on failure."""
    if not model_path:
        return {}
    try:
        from ultralytics import YOLO   # noqa: PLC0415
        model = YOLO(model_path)
        return {int(k): str(v) for k, v in model.names.items()}
    except Exception:
        return {}


def decode_pointcloud2_xyz_rgb(msg: PointCloud2):
    """
    Decode PointCloud2 → (N,3) float array [x,y,z] and (N,) uint32 array [rgb].
    Assumes fields: x(float32), y(float32), z(float32), rgb(float32/uint32).
    Returns (xyz, rgb_u32) or (None, None) on error.
    """
    # Build field offset map
    field_map = {f.name: f for f in msg.fields}
    if not all(k in field_map for k in ('x', 'y', 'z', 'rgb')):
        return None, None

    data = bytes(msg.data)
    point_step = msg.point_step
    n_points = msg.width * msg.height

    if n_points == 0 or len(data) < n_points * point_step:
        return None, None

    ox = field_map['x'].offset
    oy = field_map['y'].offset
    oz = field_map['z'].offset
    orgb = field_map['rgb'].offset

    xyz = np.zeros((n_points, 3), dtype=np.float32)
    rgb_u32 = np.zeros(n_points, dtype=np.uint32)

    for i in range(n_points):
        base = i * point_step
        xyz[i, 0] = struct.unpack_from('<f', data, base + ox)[0]
        xyz[i, 1] = struct.unpack_from('<f', data, base + oy)[0]
        xyz[i, 2] = struct.unpack_from('<f', data, base + oz)[0]
        # rgb stored as float32 with packed bytes
        rgb_f = struct.unpack_from('<f', data, base + orgb)[0]
        rgb_u32[i] = struct.unpack('<I', struct.pack('<f', rgb_f))[0]

    return xyz, rgb_u32


class SemanticMapExporter(Node):
    def __init__(self):
        super().__init__('semantic_map_exporter')

        self.declare_parameter('semantic_map_path', SEMANTIC_MAP_PATH)
        self.declare_parameter('model_path',        '')
        self.declare_parameter('min_points',        20)    # min akumulasi sebelum export
        self.declare_parameter('export_rate',       5.0)   # detik antar tulis YAML

        self.map_path   = self.get_parameter('semantic_map_path').value
        model_path      = self.get_parameter('model_path').value
        self.min_points = self.get_parameter('min_points').value
        export_rate     = self.get_parameter('export_rate').value

        # Load class names dari YOLO model (optional)
        self.class_names = _load_class_names(model_path)
        if self.class_names:
            self.get_logger().info(
                f'Class names loaded: {len(self.class_names)} classes dari {model_path}')
        else:
            self.get_logger().info(
                'Class names tidak di-load — label akan berformat class{id}_{iid}. '
                'Set param model_path untuk nama yang lebih readable.')

        # Accumulator: {instance_id → {'class_id': int, 'xs': list, 'ys': list}}
        self._instances: dict = {}
        self._lock = threading.Lock()

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(
            PointCloud2, '/semantic/pointcloud', self._cloud_cb, qos)

        # Timer export YAML
        self.create_timer(export_rate, self._export_yaml)

        self.get_logger().info(
            f'SemanticMapExporter ready | '
            f'min_points={self.min_points} | '
            f'export_rate={export_rate}s | '
            f'output={self.map_path}')

    # ─────────────────────────────────────────────────────────────
    # CLOUD CALLBACK
    # ─────────────────────────────────────────────────────────────
    def _cloud_cb(self, msg: PointCloud2):
        xyz, rgb_u32 = decode_pointcloud2_xyz_rgb(msg)
        if xyz is None:
            return

        # Decode class_id dan instance_id dari RGB
        class_ids  = ((rgb_u32 >> 16) & 0xFF).astype(np.int32)
        inst_ids   = ((rgb_u32 >> 8)  & 0xFF).astype(np.int32)

        # Filter: abaikan background (inst_id=0 atau class_id=0)
        valid = (inst_ids > 0) & (class_ids > 0)
        if not valid.any():
            return

        xs   = xyz[valid, 0]
        ys   = xyz[valid, 1]
        cids = class_ids[valid]
        iids = inst_ids[valid]

        with self._lock:
            for x, y, cid, iid in zip(xs, ys, cids, iids):
                if iid not in self._instances:
                    self._instances[iid] = {'class_id': int(cid), 'xs': [], 'ys': []}
                entry = self._instances[iid]
                entry['xs'].append(float(x))
                entry['ys'].append(float(y))
                # class_id: ambil yang paling sering (majority vote sederhana: pakai yg pertama)
                # Di-update hanya jika belum pernah di-set atau count-nya 0
                if len(entry['xs']) == 1:
                    entry['class_id'] = int(cid)

    # ─────────────────────────────────────────────────────────────
    # YAML EXPORT
    # ─────────────────────────────────────────────────────────────
    def _make_label(self, class_id: int, instance_id: int) -> str:
        name = self.class_names.get(class_id, f'class{class_id}')
        return f'{name}_{instance_id}'

    def _export_yaml(self):
        with self._lock:
            snapshot = {iid: dict(v) for iid, v in self._instances.items()}

        out = {}
        for iid, entry in snapshot.items():
            pts = entry['xs']
            if len(pts) < self.min_points:
                continue
            cid   = entry['class_id']
            label = self._make_label(cid, iid)
            x_mean = float(np.mean(entry['xs']))
            y_mean = float(np.mean(entry['ys']))
            out[label] = {
                'x':          round(x_mean, 4),
                'y':          round(y_mean, 4),
                'class_id':   cid,
                'cell_count': len(pts),
            }

        if not out:
            return

        os.makedirs(os.path.dirname(self.map_path), exist_ok=True)
        with open(self.map_path, 'w') as f:
            yaml.dump(out, f, default_flow_style=False, allow_unicode=True)

        self.get_logger().info(
            f'[Export] {len(out)} objek → {self.map_path}',
            throttle_duration_sec=10.0)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapExporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
