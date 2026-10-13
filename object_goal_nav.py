#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════╗
║  object_goal_nav.py — Object-Goal Navigation untuk Husky A200  ║
║                                                                  ║
║  Baca ~/.ros/semantic_map.yaml, terima target class name,       ║
║  lalu navigate ke instance terdekat menggunakan Nav2.           ║
║                                                                  ║
║  Interface:                                                      ║
║    Publish ke /object_goal/target (std_msgs/String)             ║
║    Contoh: ros2 topic pub /object_goal/target std_msgs/String   ║
║            "data: 'shelf'"  --once                              ║
║                                                                  ║
║  Parameter:                                                      ║
║    target_class      (str)   : Target default (default: '')     ║
║    goal_distance     (float) : Jarak berhenti dari objek (1.5m) ║
║    retry_offset_deg  (float) : Offset sudut retry (45°)        ║
║    max_retries       (int)   : Max retry (default: 3)           ║
║    semantic_map_path (str)   : Path ke YAML                     ║
║    robot_base_frame  (str)   : Frame robot base                 ║
╚══════════════════════════════════════════════════════════════════╝
"""

import os
import math
import time
import yaml
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import String

import tf2_ros
from tf2_ros import TransformException

import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'semantic'))
try:
    from room_zone_lookup import RoomZoneMap
except ImportError:
    RoomZoneMap = None


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class ObjectGoalNav(Node):
    def __init__(self):
        super().__init__('object_goal_nav')

        # ── Parameter ──────────────────────────────────────────────
        self.declare_parameter('target_class',      '')
        self.declare_parameter('goal_distance',     1.5)
        self.declare_parameter('retry_offset_deg',  45.0)
        self.declare_parameter('max_retries',       3)
        self.declare_parameter('semantic_map_path',
            os.path.expanduser('~/.ros/semantic_map.yaml'))
        self.declare_parameter('robot_base_frame',  'a200_1060/base_link')
        self.declare_parameter('zone_map_yaml_path',
            os.path.join(os.path.dirname(os.path.abspath(__file__)),
                'output/scene_zone_test/scene_zone_final_v9_final.yaml'))

        self._map_path     = self.get_parameter('semantic_map_path').value
        self._goal_dist    = self.get_parameter('goal_distance').value
        self._retry_deg    = self.get_parameter('retry_offset_deg').value
        self._max_retries  = self.get_parameter('max_retries').value
        self._base_frame   = self.get_parameter('robot_base_frame').value
        zone_yaml           = self.get_parameter('zone_map_yaml_path').value

        # ── Room-zone map (opsional — kalau gagal load, fallback ke
        # perilaku lama: offset buta tanpa cek ruangan) ────────────────
        self._zone_map = None
        if RoomZoneMap is not None and os.path.exists(zone_yaml):
            try:
                self._zone_map = RoomZoneMap(zone_yaml)
                self.get_logger().info(
                    f'RoomZoneMap loaded: {self._zone_map.n_rooms} ruangan terdeteksi')
            except Exception as e:
                self.get_logger().warn(f'Gagal load RoomZoneMap ({e}) — jalan tanpa cek ruangan.')
        else:
            self.get_logger().warn(
                f'RoomZoneMap tidak tersedia ({zone_yaml}) — jalan tanpa cek ruangan.')

        # ── TF ─────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Nav2 Action Client ─────────────────────────────────────
        self._nav_client  = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── Semantic Map ───────────────────────────────────────────
        self._map_lock     = threading.Lock()
        self._semantic_map = {}
        self._load_map()
        self.create_timer(10.0, self._load_map)   # reload periodik

        # ── Subscription target ─────────────────────────────────────
        self.create_subscription(
            String, '/object_goal/target', self._target_cb, 10)

        # ── Target dari parameter (one-shot setelah 3s) ────────────
        default_target = self.get_parameter('target_class').value
        if default_target:
            threading.Timer(
                3.0, lambda: self._navigate_to(default_target)
            ).start()

        self.get_logger().info(
            f'ObjectGoalNav ready | '
            f'goal_distance={self._goal_dist}m | '
            f'retry_offset={self._retry_deg}° | '
            f'max_retries={self._max_retries} | '
            f'Subscribe: /object_goal/target')

    # ─────────────────────────────────────────────────────────────
    # LOAD SEMANTIC MAP
    # ─────────────────────────────────────────────────────────────
    def _load_map(self):
        if not os.path.exists(self._map_path):
            self.get_logger().warn(
                f'Semantic map tidak ditemukan: {self._map_path}',
                throttle_duration_sec=30.0)
            return
        try:
            with open(self._map_path, 'r') as f:
                data = yaml.safe_load(f) or {}
            with self._map_lock:
                self._semantic_map = data
            self.get_logger().info(
                f'[Map] {len(data)} objek loaded',
                throttle_duration_sec=30.0)
        except Exception as e:
            self.get_logger().error(f'Gagal load map: {e}')

    # ─────────────────────────────────────────────────────────────
    # TARGET CALLBACK
    # ─────────────────────────────────────────────────────────────
    def _target_cb(self, msg: String):
        target = msg.data.strip()
        if not target:
            return
        self.get_logger().info(f'Target diterima: "{target}"')
        threading.Thread(
            target=self._navigate_to, args=(target,), daemon=True
        ).start()

    # ─────────────────────────────────────────────────────────────
    # GET ROBOT POSE
    # ─────────────────────────────────────────────────────────────
    def _get_robot_pose(self):
        """Return (x, y) posisi robot di map frame, atau None jika gagal."""
        try:
            t = self._tf_buffer.lookup_transform(
                'map', self._base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=2.0))
            x = t.transform.translation.x
            y = t.transform.translation.y
            return (x, y)
        except TransformException as e:
            self.get_logger().error(f'TF lookup gagal: {e}')
            return None

    # ─────────────────────────────────────────────────────────────
    # FIND CLOSEST OBJECT
    # ─────────────────────────────────────────────────────────────
    def _find_closest(self, class_name: str, robot_xy):
        """
        Cari instance terdekat dari class_name di semantic map.
        Label format: 'shelf_1', 'trash-can_3', 'vertical-ladder_2'
        Class diambil dari semua karakter sebelum '_<angka>' terakhir.
        Return (label, x, y) atau None.
        """
        rx, ry = robot_xy
        best = None
        best_dist = float('inf')

        with self._map_lock:
            for label, entry in self._semantic_map.items():
                # Parse class: ambil semua sebelum '_<digit>' terakhir
                parts = label.rsplit('_', 1)
                if len(parts) != 2 or not parts[1].isdigit():
                    continue
                obj_class = parts[0]
                if obj_class != class_name:
                    continue

                ox = float(entry.get('x', 0.0))
                oy = float(entry.get('y', 0.0))
                dist = math.hypot(rx - ox, ry - oy)
                if dist < best_dist:
                    best_dist = dist
                    best = (label, ox, oy)

        return best

    # ─────────────────────────────────────────────────────────────
    # PILIH ANGLE OFFSET (room-aware kalau RoomZoneMap tersedia)
    # ─────────────────────────────────────────────────────────────
    def _pick_angle_offsets(self, ox: float, oy: float, robot_xy) -> list:
        """
        Cari kandidat angle_offset (utk _compute_goal_pose), diurutkan:
        1) titik goal yg berada di RUANGAN YANG SAMA dgn objek target,
           diurutkan dari offset sudut terkecil (paling dekat arah alami
           "ke arah robot") ke terbesar.
        2) kalau gak ada kandidat same-room (mis. RoomZoneMap gak ke-load,
           atau objek di ruangan terisolasi), fallback ke urutan lama
           (0°, ±retry_deg, ±2*retry_deg, dst) tanpa cek ruangan sama sekali
           — drpd gagal total gak nyoba apa-apa.
        Cuma ngitung matematika murni di sini, TIDAK kirim goal apa pun ke
        Nav2 — jadi aman dipanggil sebanyak apa pun tanpa gerakkan robot.
        """
        # Kandidat sudut lebih rapat drpd list lama (tiap 20°, satu putaran
        # penuh) supaya lebih besar peluang ketemu titik yg valid di
        # ruangan yg sama, bukan cuma 5 opsi kasar.
        candidate_degs = [0.0]
        step = max(10.0, self._retry_deg / 2.0)
        deg = step
        while deg < 360.0:
            candidate_degs.append(deg)
            candidate_degs.append(-deg)
            deg += step

        if self._zone_map is None:
            # Fallback: perilaku lama, gak ada info ruangan.
            fallback = [0.0, self._retry_deg, -self._retry_deg,
                        self._retry_deg * 2, -self._retry_deg * 2]
            return fallback

        same_room_candidates = []
        other_candidates = []
        for aoff in candidate_degs:
            goal_pose = self._compute_goal_pose(ox, oy, robot_xy, aoff)
            gx = goal_pose.pose.position.x
            gy = goal_pose.pose.position.y
            if self._zone_map.same_room(gx, gy, ox, oy):
                same_room_candidates.append(aoff)
            else:
                other_candidates.append(aoff)

        if same_room_candidates:
            self.get_logger().info(
                f'RoomZoneMap: {len(same_room_candidates)}/{len(candidate_degs)} '
                f'kandidat offset berada di ruangan yg sama dgn objek — '
                f'prioritaskan itu.')
            return same_room_candidates + other_candidates

        self.get_logger().warn(
            'RoomZoneMap: TIDAK ADA kandidat offset di ruangan yang sama '
            'dgn objek (mungkin objek di ruangan terisolasi/sempit) — '
            'fallback coba semua kandidat apa adanya.')
        return candidate_degs

    # ─────────────────────────────────────────────────────────────
    # COMPUTE GOAL POSE
    # ─────────────────────────────────────────────────────────────
    def _compute_goal_pose(self, ox: float, oy: float,
                           robot_xy, angle_offset_deg: float = 0.0) -> PoseStamped:
        """
        Hitung goal pose pada jarak goal_distance dari objek.
        Arah: dari objek ke robot + angle_offset (untuk retry).
        Orientasi robot: menghadap ke objek.
        """
        rx, ry = robot_xy
        dx = rx - ox
        dy = ry - oy
        dist = math.hypot(dx, dy)

        base_angle = math.atan2(dy, dx) if dist > 0.01 else 0.0
        angle = base_angle + math.radians(angle_offset_deg)

        gx = ox + self._goal_dist * math.cos(angle)
        gy = oy + self._goal_dist * math.sin(angle)

        # Orientasi robot menghadap ke objek
        yaw = math.atan2(oy - gy, ox - gx)

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.position.z = 0.0
        goal.pose.orientation = yaw_to_quaternion(yaw)
        return goal

    # ─────────────────────────────────────────────────────────────
    # NAVIGATE TO CLASS
    # ─────────────────────────────────────────────────────────────
    def _navigate_to(self, class_name: str):
        self.get_logger().info(f'=== Navigasi ke class: "{class_name}" ===')
        self._load_map()

        # Cek Nav2 ready — tunggu lebih lama karena lifecycle manager butuh waktu
        self.get_logger().info('Menunggu Nav2 action server...')
        if not self._nav_client.wait_for_server(timeout_sec=60.0):
            self.get_logger().error('Nav2 action server tidak tersedia setelah 60s!')
            return

        # Pose robot
        robot_xy = self._get_robot_pose()
        if robot_xy is None:
            self.get_logger().error('Tidak bisa dapat pose robot dari TF')
            return

        # Cari objek di semantic map
        # Support dua mode:
        #   1. Instance label: 'vertical-ladder_51' → langsung ke instance itu
        #   2. Class name: 'vertical-ladder' → ke instance terdekat
        with self._map_lock:
            exact_match = self._semantic_map.get(class_name)

        if exact_match is not None:
            # Mode instance langsung
            ox = float(exact_match.get('x', 0.0))
            oy = float(exact_match.get('y', 0.0))
            label = class_name
            self.get_logger().info(f'Mode: instance langsung → {label}')
        else:
            # Mode class name → cari instance terdekat
            result = self._find_closest(class_name, robot_xy)
            if result is None:
                with self._map_lock:
                    available = list(self._semantic_map.keys())
                self.get_logger().error(
                    f'Class "{class_name}" tidak ditemukan di semantic map.\n'
                    f'Objek tersedia: {available}')
                return
            label, ox, oy = result
            self.get_logger().info(f'Mode: class terdekat → {label}')
        dist_to_obj = math.hypot(robot_xy[0] - ox, robot_xy[1] - oy)
        self.get_logger().info(
            f'Target: {label} @ ({ox:.2f}, {oy:.2f}) | '
            f'Robot @ ({robot_xy[0]:.2f}, {robot_xy[1]:.2f}) | '
            f'Jarak: {dist_to_obj:.2f}m | '
            f'Goal offset: {self._goal_dist}m dari objek')

        angle_offsets = self._pick_angle_offsets(ox, oy, robot_xy)

        for attempt in range(min(self._max_retries + 1, len(angle_offsets))):
            aoff = angle_offsets[attempt]
            goal_pose = self._compute_goal_pose(ox, oy, robot_xy, aoff)

            self.get_logger().info(
                f'[Attempt {attempt + 1}/{self._max_retries + 1}] '
                f'Goal: ({goal_pose.pose.position.x:.2f}, '
                f'{goal_pose.pose.position.y:.2f}) | '
                f'angle_offset={aoff:.0f}°')

            success = self._send_goal_and_wait(goal_pose)

            if success:
                self.get_logger().info(
                    f'✓ Berhasil navigasi ke {label}!')
                return

            if attempt < self._max_retries:
                self.get_logger().warn(
                    f'Attempt {attempt + 1} gagal. '
                    f'Retry dalam 2s dengan offset {angle_offsets[attempt+1]:.0f}°...')
                time.sleep(2.0)

        self.get_logger().error(
            f'✗ Semua {self._max_retries + 1} attempt gagal untuk {label}')

    # ─────────────────────────────────────────────────────────────
    # SEND GOAL & WAIT
    # ─────────────────────────────────────────────────────────────
    def _send_goal_and_wait(self, goal_pose: PoseStamped) -> bool:
        """Kirim goal ke Nav2, block sampai selesai. Return True jika sukses."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        # Kirim goal
        send_future = self._nav_client.send_goal_async(goal_msg)

        # Tunggu goal diterima (max 10s)
        deadline = time.time() + 10.0
        while not send_future.done():
            time.sleep(0.05)
            if time.time() > deadline:
                self.get_logger().error('Timeout kirim goal ke Nav2')
                return False

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Goal ditolak Nav2')
            return False

        self.get_logger().info('Goal diterima Nav2, menunggu navigasi...')

        # Tunggu hasil (max 120s)
        result_future = goal_handle.get_result_async()
        deadline = time.time() + 120.0
        while not result_future.done():
            time.sleep(0.1)
            if time.time() > deadline:
                self.get_logger().error('Nav2 timeout (120s) — cancel goal')
                goal_handle.cancel_goal_async()
                return False

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            return True

        self.get_logger().warn(f'Nav2 selesai dengan status: {status}')
        return False


def main(args=None):
    import os
    rclpy.init(args=args)
    node = ObjectGoalNav()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
