#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════╗
║  Lab Robot Detector — model CUSTOM (best_brin_yolo11m_v1), khusus║
║  3 kelas robot lab MOBILE: dell-e(13), i-seater(14), o-seater(15)║
║                                                                  ║
║  robotic-arm(16) SENGAJA DIKECUALIKAN — itu fixture diam di      ║
║  tempat, bukan objek dinamis (dikonfirmasi user).                ║
║                                                                  ║
║  Beda dari yolo_detector.py (task='segment', dipakai Fase B utk  ║
║  bangun semantic_map.yaml) — node ini task='detect' (lebih       ║
║  ringan, cuma bbox, gak butuh mask), khusus utk tracking          ║
║  posisi+kecepatan real-time saat navigasi (Fase C).               ║
║  Pakai .pt (BUKAN best_brin_yolo11m_v1_detect.engine — engine    ║
║  itu TERBUKTI RUSAK/mismatch, return class_id 48 padahal model   ║
║  cuma 17 kelas; .pt divalidasi benar via bag replay, o-seater    ║
║  terdeteksi konsisten). TensorRT engine detect-only baru perlu   ║
║  di-export ULANG di kmp-orin kalau mau performa lebih cepat.     ║
║                                                                  ║
║  Pola sama persis dgn person_detector.py (depth fusion + TF ke   ║
║  map + WorldTracker) — cuma model & filter kelas yang beda.      ║
║                                                                  ║
║  Subscribe:                                                      ║
║    /camera/camera/color/image_raw                    (rgb8)     ║
║    /camera/camera/aligned_depth_to_color/image_raw    (16UC1)   ║
║    /camera/camera/color/camera_info                             ║
║                                                                  ║
║  Publish:                                                        ║
║    /semantic/lab_robot_tracks   (String, JSON)                  ║
║      [{track_id, label, x, y, speed, moving}, ...] — frame 'map' ║
╚══════════════════════════════════════════════════════════════════╝
"""

import json
import time

import numpy as np
import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from ultralytics import YOLO

import message_filters
from world_tracker import WorldTracker, quat_to_rotation_matrix

MAP_FRAME = 'map'
# dell-e, i-seater, o-seater — robotic-arm(16) TIDAK diikutkan (fixture diam).
MOBILE_ROBOT_CLASS_IDS = [13, 14, 15]


class LabRobotDetectorNode(Node):
    def __init__(self):
        super().__init__('lab_robot_detector')

        self.declare_parameter('model_path',
            '/root/ws/project/rtabmap_allinone/models/best_brin_yolo11m_v1_20260517_0724.pt')
        # [2026-07-13] confidence 0.25 (bukan 0.5 spt person_detector) —
        # o-seater/i-seater/dell-e divalidasi kedeteksi cuma di ~0.26 saat
        # bag replay (angle/jarak wajar, bukan close-up) — 0.5 kegedean,
        # bakal miss deteksi valid. Perlu tuning ulang begitu ada data live.
        self.declare_parameter('confidence',       0.25)
        self.declare_parameter('device',           'cpu')
        self.declare_parameter('process_every_n',  3)
        self.declare_parameter('max_depth',        8.0)
        self.declare_parameter('min_depth',        0.2)
        self.declare_parameter('max_match_dist',   1.0)
        self.declare_parameter('moving_threshold', 0.15)

        model_path = self.get_parameter('model_path').value
        self.conf = self.get_parameter('confidence').value
        self.dev = self.get_parameter('device').value
        self.every_n = self.get_parameter('process_every_n').value
        self.max_d = self.get_parameter('max_depth').value
        self.min_d = self.get_parameter('min_depth').value

        self.get_logger().info(f'Loading: {model_path}')
        self.model = YOLO(model_path, task='detect')
        self.tracker = WorldTracker(
            max_match_dist=self.get_parameter('max_match_dist').value,
            moving_threshold_mps=self.get_parameter('moving_threshold').value)

        self.fx = self.fy = self.cx = self.cy = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos_be = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        qos_rel = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info',
            self._cam_info_cb, qos_rel)

        sub_color = message_filters.Subscriber(
            self, Image, '/camera/camera/color/image_raw', qos_profile=qos_be)
        sub_depth = message_filters.Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw', qos_profile=qos_be)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [sub_color, sub_depth], queue_size=10, slop=0.1)
        self.sync.registerCallback(self._sync_cb)

        self.pub_tracks = self.create_publisher(String, '/semantic/lab_robot_tracks', 10)

        self._fc = 0
        self.get_logger().info(
            f'Ready — mobile lab robots (classes {MOBILE_ROBOT_CLASS_IDS}) | '
            f'device: {self.dev} | every {self.every_n} frames')

    def _cam_info_cb(self, msg):
        if self.fx is not None:
            return
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]
        self.get_logger().info(
            f'Intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} cx={self.cx:.1f} cy={self.cy:.1f}')

    def _sync_cb(self, color_msg, depth_msg):
        self._fc += 1
        if self._fc % self.every_n != 0 or self.fx is None:
            return

        img = np.frombuffer(color_msg.data, dtype=np.uint8).reshape(
            (color_msg.height, color_msg.width, 3)).copy()
        dep = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(
            depth_msg.height, depth_msg.width).astype(np.float32) / 1000.0

        results = self.model(img, conf=self.conf, device=self.dev,
                              classes=MOBILE_ROBOT_CLASS_IDS, verbose=False)
        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            self._publish_tracks([])
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                MAP_FRAME, 'camera_color_optical_frame', color_msg.header.stamp,
                timeout=Duration(seconds=0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'[TF] lookup gagal: {e}', throttle_duration_sec=5.0)
            return
        tr, ro = transform.transform.translation, transform.transform.rotation
        R = quat_to_rotation_matrix(ro.x, ro.y, ro.z, ro.w)
        T = np.array([tr.x, tr.y, tr.z])

        h, w = dep.shape
        detections_map = []
        for i in range(len(boxes)):
            x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy()
            u, v = int((x1 + x2) / 2), int((y1 + y2) / 2)
            if not (0 <= u < w and 0 <= v < h):
                continue
            v0, v1 = max(0, v - 3), min(h, v + 4)
            u0, u1 = max(0, u - 3), min(w, u + 4)
            patch = dep[v0:v1, u0:u1]
            valid = patch[(patch > self.min_d) & (patch < self.max_d)]
            if valid.size == 0:
                continue
            Z = float(np.median(valid))

            X_cam = (u - self.cx) * Z / self.fx
            Y_cam = (v - self.cy) * Z / self.fy
            p_cam = np.array([X_cam, Y_cam, Z])
            p_map = R @ p_cam + T
            label = self.model.names.get(int(boxes.cls[i]), str(int(boxes.cls[i])))
            detections_map.append((p_map[0], p_map[1], label))

        now = time.time()
        tracked = self.tracker.update(detections_map, now)
        self._publish_tracks([
            {'track_id': tid, 'label': label, 'x': round(x, 2), 'y': round(y, 2),
             'speed': round(speed, 2), 'moving': bool(moving)}
            for (x, y, label), (tid, speed, moving) in zip(detections_map, tracked)
        ])

        if (self._fc // self.every_n) % 30 == 1:
            n_moving = sum(1 for t in tracked if t[2])
            self.get_logger().info(
                f'[Frame {self._fc}] Robots: {len(detections_map)} | '
                f'Tracks: {len(self.tracker.tracks)} | Moving: {n_moving}')

    def _publish_tracks(self, tracks):
        msg = String()
        msg.data = json.dumps(tracks)
        self.pub_tracks.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LabRobotDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
