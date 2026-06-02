#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════╗
║  Semantic Projector — Segmentation + Instance Version           ║
║                                                                  ║
║  Subscribe:                                                      ║
║    /semantic/label_image        (mono8)  — class_id+1/pixel    ║
║    /semantic/instance_image     (mono8)  — instance_id/pixel   ║
║    /camera/depth/image_rect_raw (16UC1)  — depth               ║
║    /camera/color/camera_info    — intrinsics                    ║
║                                                                  ║
║  Publish:                                                        ║
║    /semantic/pointcloud  (PointCloud2)                          ║
║      RGB encoding: R=class_id, G=instance_id, B=0              ║
║      → grid node decode class dan instance dari sini           ║
╚══════════════════════════════════════════════════════════════════╝
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import message_filters
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
import numpy as np
import struct


class SemanticProjectorNode(Node):
    def __init__(self):
        super().__init__('semantic_projector')

        self.declare_parameter('max_depth',   8.0)
        self.declare_parameter('min_depth',   0.2)
        self.declare_parameter('sample_step', 4)
        self.declare_parameter('sync_slop',   0.5)

        self.max_d = self.get_parameter('max_depth').value
        self.min_d = self.get_parameter('min_depth').value
        self.step  = self.get_parameter('sample_step').value
        slop       = self.get_parameter('sync_slop').value

        self.fx = self.fy = self.cx = self.cy = None

        qos_be = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        qos_rel = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(
            CameraInfo, '/camera/color/camera_info',
            self._cam_info_cb, qos_rel)

        sub_lbl  = message_filters.Subscriber(
            self, Image, '/semantic/label_image',    qos_profile=qos_be)
        sub_inst = message_filters.Subscriber(
            self, Image, '/semantic/instance_image', qos_profile=qos_be)
        sub_dep  = message_filters.Subscriber(
            self, Image, '/camera/depth/image_rect_raw', qos_profile=qos_be)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [sub_lbl, sub_inst, sub_dep], queue_size=10, slop=slop)
        self.sync.registerCallback(self._sync_cb)

        self.pub = self.create_publisher(
            PointCloud2, '/semantic/pointcloud', 10)

        self._sc, self._tp = 0, 0
        self.get_logger().info(
            f'SemanticProjector ready | '
            f'depth: {self.min_d}-{self.max_d}m | step: {self.step}')

    def _cam_info_cb(self, msg):
        if self.fx is not None:
            return
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]
        self.get_logger().info(
            f'Intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} '
            f'cx={self.cx:.1f} cy={self.cy:.1f}')

    def _sync_cb(self, lbl_msg, inst_msg, dep_msg):
        if self.fx is None:
            return

        lbl  = np.frombuffer(lbl_msg.data,  dtype=np.uint8).reshape(
            lbl_msg.height,  lbl_msg.width)
        inst = np.frombuffer(inst_msg.data, dtype=np.uint8).reshape(
            inst_msg.height, inst_msg.width)
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

                Z = dep[v, u]
                if Z < self.min_d or Z > self.max_d:
                    continue

                X = (u - self.cx) * Z / self.fx
                Y = (v - self.cy) * Z / self.fy

                # Encode class + instance ke RGB
                # R = class_id (0-79)
                # G = instance_id (1-254)
                # B = 0
                r = int(cls_val) - 1   # class_id 0-indexed
                g = int(inst_val)
                b = 0
                rgb_int = (r << 16) | (g << 8) | b
                rgb_f   = struct.unpack('f', struct.pack('I', rgb_int))[0]

                points.append((X, Y, Z, rgb_f))

        if not points:
            return

        self._tp += len(points)
        self.pub.publish(self._make_cloud(points, dep_msg.header))

        if self._sc % 10 == 1:
            self.get_logger().info(
                f'[Sync {self._sc}] Points: {len(points)} | Total: {self._tp}')

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
        msg.header.frame_id = 'camera_depth_optical_frame'
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
