#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════╗
║  send_nav_goal.py — Kirim satu NavigateToPose goal (frame map)  ║
║                                                                  ║
║  Untuk test manual di ./run.sh sim (atau produksi) tanpa perlu  ║
║  klik RViz. Goal ABSOLUT di frame map (lihat memory: JANGAN     ║
║  base_link-relative untuk goal >1 detik tempuh).                ║
║                                                                  ║
║  Usage:                                                          ║
║    python3 send_nav_goal.py <x> <y> [yaw_rad]                   ║
╚══════════════════════════════════════════════════════════════════╝
"""

import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


def quaternion_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class GoalSender(Node):

    def __init__(self, x, y, yaw):
        super().__init__('send_nav_goal')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._x, self._y, self._yaw = x, y, yaw
        self._done = False

    def send(self):
        self.get_logger().info('Menunggu action server navigate_to_pose...')
        self._client.wait_for_server()

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self._x
        goal.pose.pose.position.y = self._y
        qx, qy, qz, qw = quaternion_from_yaw(self._yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(f'Kirim goal: x={self._x:.3f} y={self._y:.3f} yaw={self._yaw:.3f}')
        send_future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'  jarak tersisa: {fb.distance_remaining:.2f}m')

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal ditolak.')
            self._done = True
            return
        self.get_logger().info('Goal diterima, menunggu hasil...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        status = future.result().status
        label = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }.get(status, f'status={status}')
        self.get_logger().info(f'Hasil: {label}')
        self._done = True

    @property
    def done(self):
        return self._done


def main():
    if len(sys.argv) < 3:
        print('Usage: python3 send_nav_goal.py <x> <y> [yaw_rad]')
        sys.exit(1)

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0

    rclpy.init()
    node = GoalSender(x, y, yaw)
    node.send()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
