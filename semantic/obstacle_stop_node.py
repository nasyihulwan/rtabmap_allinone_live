#!/usr/bin/env python3
"""
obstacle_stop_node.py — Lidar 3D obstacle safety stop.

Subscribe /velodyne_points (PointCloud2), cek semua titik dalam bounding
box di depan robot. Jauh lebih reliable dari 2D scan untuk deteksi kursi,
meja, dan objek tipis yang bisa lolos dari single scan ring.

Zone deteksi (frame robot: X=depan, Y=kiri):
  Forward zone : 0 < x < fwd_distance, |y| < robot_width/2, z_min < z < z_max
  Side zone    : 0 < x < side_distance, robot_width/2 < |y| < side_width, z_min < z < z_max

Parameters:
  point_topic    : PointCloud2 topic (default: /velodyne_points)
  base_frame     : Robot base frame (default: a200_1060/base_link)
  fwd_distance   : Jarak stop di depan, meter (default: 0.6)
  robot_width    : Lebar robot (full), meter (default: 0.7)
  side_distance  : Jarak cek samping, meter (default: 0.4)
  side_width     : Lebar zone samping (full dari tepi robot), meter (default: 0.3)
  z_min          : Tinggi minimum titik dianggap obstacle, meter (default: 0.05)
  z_max          : Tinggi maksimum titik dianggap obstacle, meter (default: 2.0)
  stop_hysteresis: Scan berturut sebelum trigger stop (default: 2)
  clear_hysteresis: Scan berturut sebelum clear stop (default: 3)
  publish_hz     : Rate publish emergency_stop, Hz (default: 20.0)
  verbose        : Print debug (default: false)
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

try:
    import tf2_ros
    from tf2_ros import Buffer, TransformListener
    HAS_TF = True
except ImportError:
    HAS_TF = False


def _extract_xyz(cloud: PointCloud2):
    """Vectorized extraction of finite (x,y,z) float32 arrays from a PointCloud2.

    Dulu pakai per-titik struct.unpack_from() dalam Python loop — dgn
    VLP-32C (puluhan ribu titik/scan @ 10Hz) ini bikin CPU 100%+ sustained.
    numpy slicing baca semua titik sekaligus, jauh lebih ringan.
    """
    offsets = {}
    for field in cloud.fields:
        if field.name in ('x', 'y', 'z'):
            offsets[field.name] = field.offset
    if len(offsets) < 3:
        empty = np.empty(0, dtype=np.float32)
        return empty, empty, empty

    ox, oy, oz = offsets['x'], offsets['y'], offsets['z']
    n_points = cloud.width * cloud.height
    if n_points == 0:
        empty = np.empty(0, dtype=np.float32)
        return empty, empty, empty

    raw = np.frombuffer(cloud.data, dtype=np.uint8).reshape(n_points, cloud.point_step)
    x = raw[:, ox:ox + 4].copy().view(np.float32).reshape(-1)
    y = raw[:, oy:oy + 4].copy().view(np.float32).reshape(-1)
    z = raw[:, oz:oz + 4].copy().view(np.float32).reshape(-1)

    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    return x[finite], y[finite], z[finite]


class ObstacleStopNode(Node):

    def __init__(self):
        super().__init__('obstacle_stop_node')

        # ── Parameters ─────────────────────────────────────────────────
        self.declare_parameter('point_topic',      '/velodyne_points')
        self.declare_parameter('base_frame',       'a200_1060/base_link')
        self.declare_parameter('sensor_height',    0.5)   # z offset sensor dari base_link
        self.declare_parameter('fwd_distance',     0.6)
        self.declare_parameter('robot_width',      0.7)
        self.declare_parameter('side_distance',    0.4)
        self.declare_parameter('side_width',       0.3)
        self.declare_parameter('z_min',            0.05)  # relatif terhadap lantai
        self.declare_parameter('z_max',            2.0)   # relatif terhadap lantai
        self.declare_parameter('stop_hysteresis',  2)
        self.declare_parameter('clear_hysteresis', 3)
        self.declare_parameter('publish_hz',       20.0)
        self.declare_parameter('verbose',          False)

        topic           = self.get_parameter('point_topic').value
        self._base_fr   = self.get_parameter('base_frame').value
        sensor_h        = self.get_parameter('sensor_height').value
        self._fwd_d     = self.get_parameter('fwd_distance').value
        self._half_w    = self.get_parameter('robot_width').value / 2.0
        self._side_d    = self.get_parameter('side_distance').value
        self._side_w    = self.get_parameter('side_width').value
        # z bounds relatif ke lantai → konversi ke sensor frame (z_sensor = z_floor - sensor_h)
        self._z_min     = self.get_parameter('z_min').value - sensor_h
        self._z_max     = self.get_parameter('z_max').value - sensor_h
        self._stop_h    = self.get_parameter('stop_hysteresis').value
        self._clear_h   = self.get_parameter('clear_hysteresis').value
        pub_hz          = self.get_parameter('publish_hz').value
        self._verbose   = self.get_parameter('verbose').value
        self._sensor_h  = sensor_h

        # ── TF ─────────────────────────────────────────────────────────
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._cloud_frame = None  # diisi saat pesan pertama datang

        # ── State ──────────────────────────────────────────────────────
        self._estop_active = False
        self._stop_count   = 0
        self._clear_count  = 0

        # ── Publishers ─────────────────────────────────────────────────
        self._estop_pub  = self.create_publisher(Bool,  '/a200_1060/platform/emergency_stop', 1)
        self._active_pub = self.create_publisher(Bool,  '/obstacle_stop/active', 1)
        # Zero-velocity publisher — dipakai untuk blokir Nav2 langsung
        # (emergency_stop tidak reach Husky karena DDS domain berbeda)
        self._zero_pub   = self.create_publisher(Twist, '/cmd_vel_nav', 10)

        # ── Subscriber ─────────────────────────────────────────────────
        pc_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PointCloud2, topic, self._cloud_cb, pc_qos)

        # ── Timer ──────────────────────────────────────────────────────
        self.create_timer(1.0 / pub_hz, self._publish_estop)

        z_min_floor = self.get_parameter('z_min').value
        z_max_floor = self.get_parameter('z_max').value
        self.get_logger().info(
            f'ObstacleStop 3D ready | '
            f'fwd < {self._fwd_d}m (w={self._half_w*2:.1f}m) | '
            f'side < {self._side_d}m (w={self._side_w:.1f}m) | '
            f'height {z_min_floor:.2f}-{z_max_floor:.1f}m dari lantai '
            f'(sensor frame: {self._z_min:.2f} to {self._z_max:.1f}m) | '
            f'sensor_height={sensor_h}m'
        )

    # ── Cloud callback ──────────────────────────────────────────────────
    def _cloud_cb(self, msg: PointCloud2):
        self._cloud_frame = msg.header.frame_id

        # Coba dapatkan transform cloud_frame → base_frame
        tf_mat = None
        if self._cloud_frame != self._base_fr:
            try:
                t = self._tf_buffer.lookup_transform(
                    self._base_fr, self._cloud_frame,
                    rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
                tx = t.transform.translation
                r  = t.transform.rotation
                tf_mat = (tx.x, tx.y, tx.z, r.x, r.y, r.z, r.w)
            except Exception:
                # TF belum tersedia — proses langsung di sensor frame.
                # Velodyne dipasang tegak lurus (X=depan, Y=kiri, Z=atas),
                # z bounds sudah dikoreksi ke sensor frame via sensor_height.
                pass  # tf_mat tetap None → proses in-place

        px, py, pz = _extract_xyz(msg)

        # Transform ke base_frame jika perlu (vectorized — fungsi yg sama,
        # cuma dipanggil dgn array numpy, operasinya elementwise otomatis)
        if tf_mat is not None and px.size > 0:
            px, py, pz = _transform_point(px, py, pz, tf_mat)

        if px.size == 0:
            fwd_obstacle = side_obstacle = False
            nearest_fwd = nearest_side = float('inf')
        else:
            # Filter tinggi
            z_mask = (self._z_min <= pz) & (pz <= self._z_max)
            px, py = px[z_mask], py[z_mask]
            abs_py = np.abs(py)

            # Forward zone: depan robot, lebar robot
            fwd_mask = (px > 0.0) & (px < self._fwd_d) & (abs_py < self._half_w)
            # Side zone: depan tapi lebih sempit dan dekat (exclude titik yg
            # sudah masuk forward zone — replikasi semantik elif asli)
            side_mask = (~fwd_mask) & (px > 0.0) & (px < self._side_d) & \
                        (abs_py >= self._half_w) & (abs_py < (self._half_w + self._side_w))

            fwd_obstacle  = bool(fwd_mask.any())
            side_obstacle = bool(side_mask.any())
            nearest_fwd   = float(px[fwd_mask].min()) if fwd_obstacle else float('inf')
            nearest_side  = float(px[side_mask].min()) if side_obstacle else float('inf')

        obstacle_now = fwd_obstacle or side_obstacle

        if obstacle_now:
            self._stop_count  += 1
            self._clear_count  = 0
        else:
            self._clear_count += 1
            self._stop_count   = 0

        prev = self._estop_active

        if not self._estop_active and self._stop_count >= self._stop_h:
            self._estop_active = True
            if fwd_obstacle:
                self.get_logger().warn(
                    f'OBSTACLE STOP — depan {nearest_fwd:.2f}m (threshold {self._fwd_d}m)')
            else:
                self.get_logger().warn(
                    f'OBSTACLE STOP — samping {nearest_side:.2f}m (threshold {self._side_d}m)')

        elif self._estop_active and self._clear_count >= self._clear_h:
            self._estop_active = False
            self.get_logger().info(
                f'Clear — nearest fwd={nearest_fwd:.2f}m side={nearest_side:.2f}m')

        if self._verbose and (prev != self._estop_active or obstacle_now):
            self.get_logger().info(
                f'estop={self._estop_active} fwd={nearest_fwd:.2f}m side={nearest_side:.2f}m')

    def _publish_estop(self):
        msg = Bool(data=self._estop_active)
        self._estop_pub.publish(msg)
        self._active_pub.publish(msg)
        # Ketika obstacle aktif: publish zero Twist ke /cmd_vel_nav
        # untuk override Nav2 controller output dan stop robot.
        if self._estop_active:
            self._zero_pub.publish(Twist())


def _transform_point(px, py, pz, tf_mat):
    """Apply TF (translation + quaternion rotation) to a point."""
    tx, ty, tz, qx, qy, qz, qw = tf_mat
    # Rotate
    # Using formula: v' = q * v * q^-1
    ix =  qw*px + qy*pz - qz*py
    iy =  qw*py + qz*px - qx*pz
    iz =  qw*pz + qx*py - qy*px
    iw = -qx*px - qy*py - qz*pz
    rx = ix*qw + iw*(-qx) + iy*(-qz) - iz*(-qy)
    ry = iy*qw + iw*(-qy) + iz*(-qx) - ix*(-qz)
    rz = iz*qw + iw*(-qz) + ix*(-qy) - iy*(-qx)
    return rx + tx, ry + ty, rz + tz


def main():
    rclpy.init()
    node = ObstacleStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        msg = Bool(data=False)
        node._estop_pub.publish(msg)
        node.destroy_node()


if __name__ == '__main__':
    main()
