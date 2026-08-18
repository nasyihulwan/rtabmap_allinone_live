#!/usr/bin/env python3
"""
test_semantic_projector.py — Unit tests untuk semantic_projector.py

Fokus: fusi LiDAR (decode_pointcloud2_xyz, rasterize_lidar_range).
Test tanpa ROS (mock rclpy/message_filters/tf2_ros). cv2 & numpy asli.
"""

import sys
import os
import struct
import unittest
from unittest.mock import MagicMock
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

_ROS_MOCKS = [
    'rclpy', 'rclpy.node', 'rclpy.qos', 'rclpy.duration',
    'sensor_msgs', 'sensor_msgs.msg',
    'message_filters', 'tf2_ros',
]
for _m in _ROS_MOCKS:
    sys.modules[_m] = MagicMock()

_mock_node_cls = MagicMock()
_mock_node_cls.return_value = MagicMock()
sys.modules['rclpy.node'].Node = _mock_node_cls

sys.path.insert(0, os.path.join(ROOT, 'semantic'))

from semantic_projector import (   # noqa: E402
    decode_pointcloud2_xyz, rasterize_lidar_range, NO_LIDAR,
)


def _make_velodyne_msg(points):
    """
    points: list of (x, y, z, intensity, ring, time)
    Layout persis driver Velodyne yang dikonfirmasi dari rosbag:
    point_step=22 — x,y,z,intensity float32 @ 0,4,8,12; ring uint16 @ 16;
    time float32 @ 18.
    """
    msg = MagicMock()
    fields = {}
    for name, offset in [('x', 0), ('y', 4), ('z', 8), ('intensity', 12),
                          ('ring', 16), ('time', 18)]:
        f = MagicMock()
        f.name = name
        f.offset = offset
        fields[name] = f
    msg.fields = list(fields.values())

    point_step = 22
    n = len(points)
    msg.point_step = point_step
    msg.width = n
    msg.height = 1

    data = bytearray(n * point_step)
    for i, (x, y, z, intensity, ring, t) in enumerate(points):
        base = i * point_step
        struct.pack_into('<f', data, base + 0, x)
        struct.pack_into('<f', data, base + 4, y)
        struct.pack_into('<f', data, base + 8, z)
        struct.pack_into('<f', data, base + 12, intensity)
        struct.pack_into('<H', data, base + 16, ring)
        struct.pack_into('<f', data, base + 18, t)
    msg.data = bytes(data)
    return msg


class TestDecodePointcloud2Xyz(unittest.TestCase):

    def test_decode_velodyne_layout(self):
        """Layout 22-byte (bukan 16-byte semantic/pointcloud) harus jalan
        karena decode-nya field-offset driven, bukan hardcode point_step."""
        pts = [
            (1.0, 2.0, 0.5, 100.0, 5, 0.001),
            (3.0, 4.0, 0.5, 80.0, 10, 0.002),
        ]
        msg = _make_velodyne_msg(pts)
        xyz, ok = decode_pointcloud2_xyz(msg)
        self.assertTrue(ok)
        self.assertEqual(xyz.shape, (2, 3))
        np.testing.assert_allclose(xyz[:, 0], [1.0, 3.0], atol=1e-4)
        np.testing.assert_allclose(xyz[:, 2], [0.5, 0.5], atol=1e-4)

    def test_drops_nan_points(self):
        """Driver Velodyne kadang isi titik invalid dengan NaN — harus dibuang."""
        pts = [
            (1.0, 2.0, 0.5, 100.0, 5, 0.001),
            (float('nan'), 4.0, 0.5, 80.0, 10, 0.002),
        ]
        msg = _make_velodyne_msg(pts)
        xyz, ok = decode_pointcloud2_xyz(msg)
        self.assertTrue(ok)
        self.assertEqual(xyz.shape, (1, 3))

    def test_missing_xyz_field(self):
        msg = _make_velodyne_msg([(1.0, 2.0, 0.5, 100.0, 5, 0.001)])
        msg.fields = [f for f in msg.fields if f.name != 'z']
        xyz, ok = decode_pointcloud2_xyz(msg)
        self.assertFalse(ok)
        self.assertIsNone(xyz)

    def test_empty_cloud(self):
        msg = _make_velodyne_msg([])
        xyz, ok = decode_pointcloud2_xyz(msg)
        self.assertFalse(ok)


class TestRasterizeLidarRange(unittest.TestCase):
    """fx=fy=100, cx=cy=50, image 100x100 — angka bulat supaya gampang dicek tangan."""

    FX = FY = 100.0
    CX = CY = 50.0
    H = W = 100

    def test_single_point_projects_to_expected_pixel(self):
        # X=0,Y=0,Z=2 → u=cx=50, v=cy=50
        xyz_cam = np.array([[0.0, 0.0, 2.0]])
        img = rasterize_lidar_range(
            xyz_cam, self.FX, self.FY, self.CX, self.CY, self.H, self.W,
            min_d=0.2, max_d=8.0, dilate_px=0)
        self.assertIsNotNone(img)
        self.assertAlmostEqual(float(img[50, 50]), 2.0, places=4)
        self.assertEqual(img[0, 0], NO_LIDAR)

    def test_out_of_range_points_excluded(self):
        xyz_cam = np.array([[0.0, 0.0, 100.0]])   # jauh di atas max_d
        img = rasterize_lidar_range(
            xyz_cam, self.FX, self.FY, self.CX, self.CY, self.H, self.W,
            min_d=0.2, max_d=8.0, dilate_px=0)
        self.assertIsNone(img)

    def test_collision_nearest_point_wins(self):
        """Dua titik jatuh di piksel yang sama — yang lebih dekat (Z kecil) menang."""
        xyz_cam = np.array([
            [0.0, 0.0, 5.0],
            [0.0, 0.0, 2.0],
        ])
        img = rasterize_lidar_range(
            xyz_cam, self.FX, self.FY, self.CX, self.CY, self.H, self.W,
            min_d=0.2, max_d=8.0, dilate_px=0)
        self.assertAlmostEqual(float(img[50, 50]), 2.0, places=4)

    def test_dilation_spreads_to_neighbors_but_not_far_pixels(self):
        """
        [regresi] Verifikasi eksplisit temuan empiris: setelah cv2.erode,
        piksel jauh dari titik LiDAR manapun harus tetap terbaca sebagai
        NO_LIDAR (bukan finite-tapi-salah akibat clipping np.inf ke
        float32-max oleh cv2). Ini persis bug yang ditemukan sebelum kode
        final ditulis — tanpa sentinel NO_LIDAR, test ini akan gagal.
        """
        xyz_cam = np.array([[0.0, 0.0, 2.0]])   # → pixel (50,50)
        img = rasterize_lidar_range(
            xyz_cam, self.FX, self.FY, self.CX, self.CY, self.H, self.W,
            min_d=0.2, max_d=8.0, dilate_px=5)

        # Tetangga dekat (dalam radius kernel) ikut ter-isi via dilasi.
        self.assertAlmostEqual(float(img[51, 51]), 2.0, places=4)
        # Piksel jauh (sudut citra) TIDAK boleh ikut ter-isi, dan harus
        # tetap persis NO_LIDAR — bukan sekadar "sangat besar".
        self.assertEqual(float(img[0, 0]), float(NO_LIDAR))

    def test_no_dilation_only_exact_pixel_covered(self):
        xyz_cam = np.array([[0.0, 0.0, 2.0]])
        img = rasterize_lidar_range(
            xyz_cam, self.FX, self.FY, self.CX, self.CY, self.H, self.W,
            min_d=0.2, max_d=8.0, dilate_px=0)
        self.assertAlmostEqual(float(img[50, 50]), 2.0, places=4)
        self.assertEqual(img[51, 51], NO_LIDAR)

    def test_no_valid_points_returns_none(self):
        xyz_cam = np.zeros((0, 3))
        img = rasterize_lidar_range(
            xyz_cam, self.FX, self.FY, self.CX, self.CY, self.H, self.W,
            min_d=0.2, max_d=8.0, dilate_px=5)
        self.assertIsNone(img)


def main():
    print('test_semantic_projector.py — unit tests\n')
    loader = unittest.TestLoader()
    suite = unittest.TestSuite([
        loader.loadTestsFromTestCase(TestDecodePointcloud2Xyz),
        loader.loadTestsFromTestCase(TestRasterizeLidarRange),
    ])
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)


if __name__ == '__main__':
    main()
