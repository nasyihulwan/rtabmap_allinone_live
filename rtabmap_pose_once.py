#!/usr/bin/env python3
"""
rtabmap_pose_once.py — Ambil satu pose NYATA dari RTABMap lalu keluar.

Subscribe /localization_pose, tunggu sampai pose loop-closure nyata diterima
(bukan pose awal 0,0,0 yang di-publish RTABMap saat startup), print "x y yaw"
ke stdout, lalu shutdown.

Usage:
    python3 rtabmap_pose_once.py [--timeout 90] [--min-dist 0.05]
Output (stdout):
    1.2345 -0.6789 0.3141
"""

import sys
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


def quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PoseOnce(Node):
    def __init__(self, min_dist: float, grace_secs: float):
        super().__init__('rtabmap_pose_once')
        self._done = False
        self._min_dist = min_dist
        # Hanya terima pesan yang di-publish SETELAH grace period ini.
        # Cegah pesan initial_pose 0,0,0 yang sudah ada di buffer saat subscribe.
        self._start_time = self.get_clock().now()
        self._grace = grace_secs
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/localization_pose',
            self._cb,
            10,
        )
        self.get_logger().info(
            f'Menunggu loop closure nyata (min_dist={min_dist}m, grace={grace_secs}s)...')

    def _cb(self, msg: PoseWithCovarianceStamped):
        if self._done:
            return

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        dist = math.sqrt(p.x ** 2 + p.y ** 2)
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        # Abaikan pesan yang datang terlalu cepat (pose awal/buffer dari startup)
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        if elapsed < self._grace:
            self.get_logger().info(
                f'  skip pose terlalu cepat ({elapsed:.1f}s < {self._grace}s grace): '
                f'x={p.x:.3f} y={p.y:.3f}')
            return

        # Abaikan pose yang terlalu dekat origin (bukan loop closure nyata)
        if dist < self._min_dist and abs(yaw) < 0.05:
            self.get_logger().info(
                f'  skip pose origin (dist={dist:.4f}m < {self._min_dist}m): '
                f'x={p.x:.3f} y={p.y:.3f} yaw={yaw:.3f}')
            return

        self._done = True
        print(f'{p.x:.6f} {p.y:.6f} {yaw:.6f}', flush=True)
        rclpy.shutdown()


def main():
    timeout = 90
    min_dist = 0.05   # meter — pose harus > ini dari origin agar diterima
    grace = 3.0       # detik — abaikan pesan dalam window ini setelah subscribe

    args = sys.argv[1:]
    for i, arg in enumerate(args):
        if arg == '--timeout' and i + 1 < len(args):
            timeout = int(args[i + 1])
        elif arg == '--min-dist' and i + 1 < len(args):
            min_dist = float(args[i + 1])
        elif arg == '--grace' and i + 1 < len(args):
            grace = float(args[i + 1])

    rclpy.init()
    node = PoseOnce(min_dist=min_dist, grace_secs=grace)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
