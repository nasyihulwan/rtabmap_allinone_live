#!/usr/bin/env python3
"""
test_yolo_tracker.py — Unit tests untuk SimpleIoUTracker (yolo_detector.py)

Test tanpa ROS/ultralytics (mock modules yang tidak tersedia di dev env).
Fokus: klaim "bebas dari kesalahan penggabungan intra-frame" yang disebut
proposal skripsi subbab 3.4.4 — sebelumnya tidak punya test sama sekali.
"""

import sys
import os
import unittest
from unittest.mock import MagicMock
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ── Mock modules yang tidak tersedia / butuh ROS ──────────────────────────
_MOCKS = [
    'rclpy', 'rclpy.node', 'rclpy.qos',
    'sensor_msgs', 'sensor_msgs.msg',
    'std_msgs', 'std_msgs.msg',
    'ultralytics',
]
for _m in _MOCKS:
    sys.modules[_m] = MagicMock()

_mock_node_cls = MagicMock()
_mock_node_cls.return_value = MagicMock()
sys.modules['rclpy.node'].Node = _mock_node_cls
sys.modules['ultralytics'].YOLO = MagicMock()

sys.path.insert(0, os.path.join(ROOT, 'semantic'))

from yolo_detector import SimpleIoUTracker, _build_detections_payload   # noqa: E402


def _mask(h, w, y0, y1, x0, x1):
    m = np.zeros((h, w), dtype=bool)
    m[y0:y1, x0:x1] = True
    return m


class TestSimpleIoUTrackerIntraFrame(unittest.TestCase):
    """
    [dummy test] Dua deteksi BARU dengan IoU tinggi dalam SATU frame yang
    sama harus tetap dapat instance_id terpisah (tidak digabung).
    Tracker hanya boleh mencocokkan terhadap track dari frame-frame
    SEBELUMNYA (existing_tracks snapshot), bukan sesama deteksi baru
    dalam batch update() yang sama.
    """

    def test_two_overlapping_new_masks_same_frame_get_distinct_ids(self):
        tracker = SimpleIoUTracker(iou_threshold=0.3, max_age=10)
        # Dua mask kelas sama, tumpang-tindih tinggi (IoU besar) — kasus
        # yang paling rawan salah digabung kalau tracker naif.
        m1 = _mask(100, 100, 10, 60, 10, 60)
        m2 = _mask(100, 100, 12, 62, 12, 62)

        ids = tracker.update([m1, m2], [0, 0])

        self.assertEqual(len(ids), 2)
        self.assertNotEqual(ids[0], ids[1],
            'dua deteksi baru di frame yang sama tidak boleh dapat ID sama')

    def test_many_overlapping_new_masks_same_frame_all_distinct(self):
        tracker = SimpleIoUTracker(iou_threshold=0.3, max_age=10)
        masks = [_mask(100, 100, 10 + i, 60 + i, 10 + i, 60 + i) for i in range(5)]
        ids = tracker.update(masks, [0] * 5)
        self.assertEqual(len(ids), len(set(ids)),
            'semua deteksi baru dalam 1 frame harus dapat ID unik')


class TestSimpleIoUTrackerAcrossFrames(unittest.TestCase):

    def test_same_mask_next_frame_keeps_id(self):
        tracker = SimpleIoUTracker(iou_threshold=0.3, max_age=10)
        m = _mask(100, 100, 10, 60, 10, 60)
        ids_f1 = tracker.update([m], [0])
        ids_f2 = tracker.update([m], [0])
        self.assertEqual(ids_f1[0], ids_f2[0])

    def test_different_class_same_position_gets_new_id(self):
        tracker = SimpleIoUTracker(iou_threshold=0.3, max_age=10)
        m = _mask(100, 100, 10, 60, 10, 60)
        ids_f1 = tracker.update([m], [0])
        ids_f2 = tracker.update([m], [1])   # kelas beda, posisi identik
        self.assertNotEqual(ids_f1[0], ids_f2[0])

    def test_track_expires_after_max_age(self):
        tracker = SimpleIoUTracker(iou_threshold=0.3, max_age=2)
        m = _mask(100, 100, 10, 60, 10, 60)
        first_id = tracker.update([m], [0])[0]

        # Beberapa frame kosong sampai track expired
        for _ in range(3):
            tracker.update([], [])

        new_id = tracker.update([m], [0])[0]
        self.assertNotEqual(first_id, new_id)


class TestBuildDetectionsPayload(unittest.TestCase):
    """[J2] Payload /yolo/detections — bbox+class+confidence+tracker_id mentah."""

    def test_basic_fields(self):
        payload = _build_detections_payload(
            cls_ids=[0, 2], inst_ids=[5, 7], confs=[0.831, 0.512],
            bboxes=[[10.111, 20.222, 30.333, 40.444], [1.0, 2.0, 3.0, 4.0]],
            class_names={0: 'cone', 2: 'chair'})
        self.assertEqual(len(payload), 2)
        self.assertEqual(payload[0], {
            'class_id': 0, 'class_name': 'cone', 'confidence': 0.831,
            'tracker_id': 5, 'bbox_xyxy': [10.11, 20.22, 30.33, 40.44],
        })
        self.assertEqual(payload[1]['class_name'], 'chair')
        self.assertEqual(payload[1]['tracker_id'], 7)

    def test_unknown_class_id_falls_back_to_str(self):
        payload = _build_detections_payload(
            cls_ids=[99], inst_ids=[1], confs=[0.9],
            bboxes=[[0.0, 0.0, 1.0, 1.0]], class_names={0: 'cone'})
        self.assertEqual(payload[0]['class_name'], '99')

    def test_empty_input(self):
        payload = _build_detections_payload([], [], [], [], {})
        self.assertEqual(payload, [])

    def test_json_serializable(self):
        """Payload ini langsung di-json.dumps() di callback() — pastikan
        tidak ada tipe non-serializable (numpy scalar dsb) yang lolos."""
        import json
        payload = _build_detections_payload(
            cls_ids=[0], inst_ids=[1], confs=[0.75],
            bboxes=[[1.0, 2.0, 3.0, 4.0]], class_names={0: 'cone'})
        json.dumps(payload)   # raise kalau ada tipe yang tidak bisa di-serialize


def main():
    print('test_yolo_tracker.py — unit tests SimpleIoUTracker\n')
    loader = unittest.TestLoader()
    suite = unittest.TestSuite([
        loader.loadTestsFromTestCase(TestSimpleIoUTrackerIntraFrame),
        loader.loadTestsFromTestCase(TestSimpleIoUTrackerAcrossFrames),
        loader.loadTestsFromTestCase(TestBuildDetectionsPayload),
    ])
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)


if __name__ == '__main__':
    main()
