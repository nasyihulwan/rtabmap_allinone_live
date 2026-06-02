#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════╗
║  semantic_map_viz.py — Semantic Map Visualizer                  ║
║                                                                  ║
║  Load posisi objek dari ~/.ros/semantic_map.yaml                ║
║  Publish MarkerArray ke RViz:                                   ║
║    - Sphere per objek (warna per class)                         ║
║    - Text label per objek                                       ║
║                                                                  ║
║  Publish:                                                        ║
║    /semantic/object_markers  (MarkerArray)                      ║
╚══════════════════════════════════════════════════════════════════╝
"""

import os
import yaml
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

SEMANTIC_MAP_PATH = os.path.expanduser('~/.ros/semantic_map.yaml')

# Warna per class — sama dengan capture_semantic_map.py
CLASS_COLORS_RGB = [
    (255,0,0),(0,0,255),(0,255,0),(255,165,0),(128,0,128),
    (0,255,255),(255,20,147),(139,69,19),(0,128,128),(255,215,0),
    (70,130,180),(34,139,34),(220,20,60),(148,0,211),(64,224,208),
    (255,140,0),(0,191,255),(50,205,50),(255,105,180),(188,143,143),
]


class SemanticMapViz(Node):
    def __init__(self):
        super().__init__('semantic_map_viz')

        self.declare_parameter('semantic_map_path', SEMANTIC_MAP_PATH)
        self.declare_parameter('map_frame',         'map')
        self.declare_parameter('publish_rate',       1.0)
        self.declare_parameter('sphere_size',        0.3)
        self.declare_parameter('text_size',          0.25)

        self.map_path    = self.get_parameter('semantic_map_path').value
        self.map_frame   = self.get_parameter('map_frame').value
        pub_rate         = self.get_parameter('publish_rate').value
        self.sphere_size = self.get_parameter('sphere_size').value
        self.text_size   = self.get_parameter('text_size').value

        self.semantic_map = self._load_semantic_map()

        self._marker_pub = self.create_publisher(
            MarkerArray, '/semantic/object_markers', 10)

        # Reload map tiap 10 detik supaya up-to-date
        self.create_timer(1.0 / pub_rate, self._publish_markers)
        self.create_timer(10.0,           self._reload_map)

        self.get_logger().info(
            f'SemanticMapViz ready | '
            f'{len(self.semantic_map)} objects | '
            f'frame: {self.map_frame} | '
            f'rate: {pub_rate}Hz')

    # ─────────────────────────────────────────────────────────────
    # LOAD / RELOAD
    # ─────────────────────────────────────────────────────────────
    def _load_semantic_map(self) -> dict:
        if not os.path.exists(self.map_path):
            self.get_logger().warn(f'Semantic map tidak ditemukan: {self.map_path}')
            return {}
        with open(self.map_path, 'r') as f:
            data = yaml.safe_load(f) or {}
        return data

    def _reload_map(self):
        new_map = self._load_semantic_map()
        if len(new_map) != len(self.semantic_map):
            self.get_logger().info(
                f'[Reload] {len(self.semantic_map)} → {len(new_map)} objects')
        self.semantic_map = new_map

    # ─────────────────────────────────────────────────────────────
    # PUBLISH MARKERS
    # ─────────────────────────────────────────────────────────────
    def _publish_markers(self):
        if not self.semantic_map:
            return

        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        for idx, (label, obj) in enumerate(self.semantic_map.items()):
            cid  = obj.get('class_id', 0)
            r, g, b = CLASS_COLORS_RGB[cid % len(CLASS_COLORS_RGB)]

            # ── Sphere marker ──────────────────────────────────────
            sphere = Marker()
            sphere.header.frame_id   = self.map_frame
            sphere.header.stamp      = now
            sphere.ns                = 'objects'
            sphere.id                = idx * 2
            sphere.type              = Marker.SPHERE
            sphere.action            = Marker.ADD
            sphere.pose.position.x   = obj['x']
            sphere.pose.position.y   = obj['y']
            sphere.pose.position.z   = 0.3
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = self.sphere_size
            sphere.color.r           = r / 255.0
            sphere.color.g           = g / 255.0
            sphere.color.b           = b / 255.0
            sphere.color.a           = 0.9
            sphere.lifetime.sec      = 2

            # ── Text marker ────────────────────────────────────────
            text = Marker()
            text.header.frame_id     = self.map_frame
            text.header.stamp        = now
            text.ns                  = 'labels'
            text.id                  = idx * 2 + 1
            text.type                = Marker.TEXT_VIEW_FACING
            text.action              = Marker.ADD
            text.pose.position.x     = obj['x']
            text.pose.position.y     = obj['y']
            text.pose.position.z     = 0.65
            text.pose.orientation.w  = 1.0
            text.scale.z             = self.text_size
            text.color.r = text.color.g = text.color.b = 1.0
            text.color.a             = 1.0
            text.text                = label
            text.lifetime.sec        = 2

            marker_array.markers.append(sphere)
            marker_array.markers.append(text)

        self._marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
