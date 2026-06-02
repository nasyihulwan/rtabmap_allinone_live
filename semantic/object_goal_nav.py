#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════╗
║  object_goal_nav.py — Object-Goal Navigation                    ║
║                                                                  ║
║  Load posisi objek dari ~/.ros/semantic_map.yaml                ║
║  Terima input nama objek → kirim NavigateToPose ke Nav2         ║
║                                                                  ║
║  Usage:                                                          ║
║    ros2 run ... object_goal_nav.py                              ║
║    → ketik nama objek: "chair_2", "tv_4", dst                   ║
║                                                                  ║
║  Atau via topic:                                                 ║
║    ros2 topic pub /object_goal std_msgs/String "data: chair_2"  ║
╚══════════════════════════════════════════════════════════════════╝
"""

import os
import yaml
import math
import rclpy
import threading
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

SEMANTIC_MAP_PATH = os.path.expanduser('~/.ros/semantic_map.yaml')


class ObjectGoalNav(Node):
    def __init__(self):
        super().__init__('object_goal_nav')

        self.declare_parameter('semantic_map_path', SEMANTIC_MAP_PATH)
        self.declare_parameter('map_frame',         'map')
        self.declare_parameter('goal_tolerance',    0.3)   # meter, jarak berhenti dari objek
        self.declare_parameter('max_retry',          5)     # max retry jika goal gagal
        self.declare_parameter('retry_offset',       0.5)   # offset per retry (meter)

        # Map boundary dari rtabmap_15april.yaml
        # origin: [-8.07391, -5.30406], size: 348x669 cells @ 0.05m
        self.declare_parameter('map_origin_x',  -8.07391)
        self.declare_parameter('map_origin_y',  -5.30406)
        self.declare_parameter('map_width',     348)
        self.declare_parameter('map_height',    669)
        self.declare_parameter('map_resolution', 0.05)

        self.map_path  = self.get_parameter('semantic_map_path').value
        self.map_frame = self.get_parameter('map_frame').value
        self.tolerance  = self.get_parameter('goal_tolerance').value
        self.max_retry  = self.get_parameter('max_retry').value
        self.retry_offset = self.get_parameter('retry_offset').value

        # Hitung batas map
        ox = self.get_parameter('map_origin_x').value
        oy = self.get_parameter('map_origin_y').value
        res = self.get_parameter('map_resolution').value
        w   = self.get_parameter('map_width').value
        h   = self.get_parameter('map_height').value
        self.map_x_min = ox
        self.map_x_max = ox + w * res
        self.map_y_min = oy
        self.map_y_max = oy + h * res
        self.get_logger().info(
            f'Map boundary: x=[{self.map_x_min:.2f}, {self.map_x_max:.2f}] '
            f'y=[{self.map_y_min:.2f}, {self.map_y_max:.2f}]')

        # Load semantic map
        self.semantic_map = self._load_semantic_map()

        # Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscribe ke /object_goal untuk input via topic
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(String, '/object_goal', self._goal_cb, qos)

        self.get_logger().info(
            f'ObjectGoalNav ready | '
            f'{len(self.semantic_map)} objects loaded | '
            f'frame: {self.map_frame}')
        self._print_available_objects()

        # Stuck detection
        self._current_label   = None
        self._current_retry   = 0
        self._current_orig_x  = None
        self._current_orig_y  = None
        self._feedback_count  = 0
        self._nonzero_dist    = False
        self._navigating      = False   # True saat robot sedang bergerak
        self._retry_pending   = False   # True saat retry sudah di-schedule

        # Thread untuk input CLI
        self._input_thread = threading.Thread(
            target=self._cli_input_loop, daemon=True)
        self._input_thread.start()

        # Timer cek stuck setiap 3 detik
        self.create_timer(3.0, self._check_stuck)

    # ─────────────────────────────────────────────────────────────
    # LOAD SEMANTIC MAP
    # ─────────────────────────────────────────────────────────────
    def _load_semantic_map(self) -> dict:
        if not os.path.exists(self.map_path):
            self.get_logger().error(f'Semantic map tidak ditemukan: {self.map_path}')
            return {}
        with open(self.map_path, 'r') as f:
            data = yaml.safe_load(f) or {}
        self.get_logger().info(f'Loaded {len(data)} objects dari {self.map_path}')
        return data

    def _print_available_objects(self):
        if not self.semantic_map:
            self.get_logger().warn('Semantic map kosong!')
            return
        self.get_logger().info('Objek tersedia:')
        for label in sorted(self.semantic_map.keys()):
            obj = self.semantic_map[label]
            self.get_logger().info(
                f'  {label:25s} x={obj["x"]:7.3f}  y={obj["y"]:7.3f}  '
                f'cells={obj["cell_count"]}')

    # ─────────────────────────────────────────────────────────────
    # INPUT HANDLERS
    # ─────────────────────────────────────────────────────────────
    def _goal_cb(self, msg: String):
        """Callback dari /object_goal topic."""
        self.navigate_to_object(msg.data.strip())

    def _cli_input_loop(self):
        """Input nama objek dari terminal."""
        print('\n[ObjectGoalNav] Ketik nama objek untuk navigasi (ctrl+c untuk keluar):')
        print('[ObjectGoalNav] Contoh: chair_2, tv_4, refrigerator_10\n')
        while True:
            try:
                target = input('>> Target: ').strip()
                if target:
                    self.navigate_to_object(target)
            except (EOFError, KeyboardInterrupt):
                break

    # ─────────────────────────────────────────────────────────────
    # MAP BOUNDARY
    # ─────────────────────────────────────────────────────────────
    def _clamp_to_map(self, x: float, y: float):
        """Clamp koordinat ke dalam batas map dengan margin."""
        margin = 0.5  # meter dari tepi map
        cx = max(self.map_x_min + margin, min(self.map_x_max - margin, x))
        cy = max(self.map_y_min + margin, min(self.map_y_max - margin, y))
        return cx, cy

    def _is_in_map(self, x: float, y: float) -> bool:
        return (self.map_x_min <= x <= self.map_x_max and
                self.map_y_min <= y <= self.map_y_max)

    # ─────────────────────────────────────────────────────────────
    # NAVIGATE
    # ─────────────────────────────────────────────────────────────
    def _offset_toward_center(self, x: float, y: float, step: int) -> tuple:
        """Offset kecil ke berbagai arah dari koordinat asli.
        step 1-5 → (atas, bawah, kanan, kiri, atas-jauh)
        """
        offsets = [
            (0.0, self.retry_offset),   # step 1: atas
            (0.0, -self.retry_offset),   # step 2: bawah
            (self.retry_offset, 0.0),   # step 3: kanan
            (-self.retry_offset, 0.0),   # step 4: kiri
            (0.0, self.retry_offset * 2),   # step 5: atas jauh
        ]
        idx = (step - 1) % len(offsets)
        dx, dy = offsets[idx]
        return x + dx, y + dy

    def navigate_to_object(self, label: str, retry: int = 0, orig_x: float = None, orig_y: float = None):
        """Lookup posisi objek dan kirim goal ke Nav2. Retry dengan offset jika gagal."""

        # Reload map supaya selalu up-to-date
        self.semantic_map = self._load_semantic_map()

        if label not in self.semantic_map:
            self.get_logger().error(f'Objek "{label}" tidak ditemukan di semantic map.')
            self._print_available_objects()
            return

        obj = self.semantic_map[label]
        x, y = obj['x'], obj['y']

        # Simpan koordinat original untuk retry
        if orig_x is None:
            orig_x, orig_y = x, y

        # Clamp ke dalam map jika di luar boundary
        if not self._is_in_map(x, y):
            cx, cy = self._clamp_to_map(x, y)
            self.get_logger().warn(
                f'Objek "{label}" di luar map ({x:.2f}, {y:.2f}) → '
                f'clamp ke ({cx:.2f}, {cy:.2f})')
            x, y = cx, cy

        # Offset ke arah tengah map jika ini retry
        if retry > 0:
            x, y = self._offset_toward_center(orig_x, orig_y, retry)
            x, y = self._clamp_to_map(x, y)
            self.get_logger().warn(
                f'[Retry {retry}/{self.max_retry}] "{label}" → '
                f'offset ke ({x:.2f}, {y:.2f})')

        self.get_logger().info(
            f'→ Navigasi ke "{label}" | x={x:.3f} y={y:.3f}')

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server tidak tersedia!')
            return

        goal_msg            = NavigateToPose.Goal()
        goal_msg.pose       = PoseStamped()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp    = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        # Simpan context untuk stuck detection
        self._current_label  = label
        self._current_retry  = retry
        self._current_orig_x = orig_x
        self._current_orig_y = orig_y
        self._feedback_count = 0
        self._nonzero_dist   = False

        send_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_cb)
        send_future.add_done_callback(
            lambda f: self._goal_response_cb(f, label, retry, orig_x, orig_y))

    def _goal_response_cb(self, future, label: str, retry: int = 0,
                          orig_x: float = None, orig_y: float = None):
        goal_handle = future.result()
        if not goal_handle.accepted:
            if retry < self.max_retry:
                self.get_logger().warn(
                    f'[Rejected] Goal ke "{label}" ditolak Nav2 → '
                    f'retry {retry+1}/{self.max_retry} dalam 2 detik...')
                self._schedule_retry(label, retry+1, orig_x, orig_y, delay=2.0)
            else:
                self.get_logger().error(
                    f'✗ "{label}" tidak reachable setelah {self.max_retry} retry.')
            return
        self._navigating = True
        self.get_logger().info(f'Goal ke "{label}" diterima, navigasi dimulai...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._result_cb(f, label, retry, orig_x, orig_y))

    def _feedback_cb(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self._feedback_count += 1
        if dist > 0.1:
            self._nonzero_dist = True
        self.get_logger().info(
            f'  Sisa jarak: {dist:.2f}m',
            throttle_duration_sec=2.0)

    def _schedule_retry(self, label, retry, orig_x, orig_y, delay=2.0):
        """Schedule retry setelah delay detik — one-shot via flag."""
        self._retry_pending = True
        fired = [False]

        def _do_retry():
            if fired[0]:
                return
            fired[0] = True
            self._retry_pending = False
            self.navigate_to_object(label, retry=retry,
                                    orig_x=orig_x, orig_y=orig_y)

        self.create_timer(delay, _do_retry)

    def _check_stuck(self):
        """Deteksi robot stuck: feedback sudah > 3x tapi distance selalu 0."""
        if self._current_label is None:
            return
        if self._navigating or self._retry_pending:
            return
        if self._feedback_count >= 3 and not self._nonzero_dist:
            label   = self._current_label
            retry   = self._current_retry
            orig_x  = self._current_orig_x
            orig_y  = self._current_orig_y
            self._current_label = None
            if retry < self.max_retry:
                self.get_logger().warn(
                    f'[Stuck] Robot tidak bergerak ke "{label}" → '
                    f'retry {retry+1}/{self.max_retry} dalam 2 detik...')
                self._schedule_retry(label, retry+1, orig_x, orig_y, delay=2.0)
            else:
                self.get_logger().error(
                    f'✗ "{label}" tidak reachable setelah {self.max_retry} retry.')

    def _result_cb(self, future, label: str, retry: int = 0,
                   orig_x: float = None, orig_y: float = None):
        result = future.result()
        status = result.status
        if status == 4:  # SUCCEEDED
            self._navigating = False
            self._current_label = None
            self.get_logger().info(f'✓ Berhasil navigasi ke "{label}"!')
        else:
            self._navigating = False
            if retry < self.max_retry:
                self.get_logger().warn(
                    f'✗ Navigasi ke "{label}" gagal (status={status}) → '
                    f'retry {retry+1}/{self.max_retry} dalam 2 detik...')
                self._schedule_retry(label, retry+1, orig_x, orig_y, delay=2.0)
            else:
                self.get_logger().error(
                    f'✗ Navigasi ke "{label}" gagal setelah {self.max_retry} retry.')


def main(args=None):
    rclpy.init(args=args)
    node = ObjectGoalNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
