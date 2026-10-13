#!/usr/bin/env python3
"""
test_room_feature_extractor.py — Tests utk room_feature_extractor.py /
room_type_classifier.py (klasifikasi jenis ruangan via SegFormer+YOLO
teragregasi per-ruangan).

Sengaja dipisah jadi 2 kelompok (sama seperti test_segformer.py):
  - Pure numpy/scipy/sklearn: jalan di mesin manapun tanpa ROS2/GPU/bag/db
    (RoomZoneMap, RoomHistogramAccumulator, LOOCV wiring di room_type_classifier).
  - GPU-gated end-to-end: skip cleanly kalau torch atau bag/db/model tidak
    ada — cuma jalan penuh di mesin yg punya semuanya (Orin).

Run:
  python3 tests/test_room_feature_extractor.py
  python3 tests/test_room_feature_extractor.py -v
"""

import os
import sys
import tempfile
import unittest

import numpy as np
import yaml
from PIL import Image

ROOT         = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SEMANTIC_DIR = os.path.join(ROOT, 'semantic')
sys.path.insert(0, SEMANTIC_DIR)

from room_zone_lookup import RoomZoneMap
from room_feature_extractor import RoomHistogramAccumulator

try:
    from sklearn.linear_model import LogisticRegression
    HAS_SKLEARN = True
except ImportError:
    HAS_SKLEARN = False

# fuse_room_tag/fuse_all_rooms/build_feature_matrix TIDAK butuh sklearn (import
# ditunda di dalam room_type_classifier.py) -- selalu bisa diimport di sini.
from room_type_classifier import build_feature_matrix, fuse_room_tag, fuse_all_rooms

if HAS_SKLEARN:
    from room_type_classifier import run_loocv

try:
    import torch  # noqa: F401
    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False

# Contoh bag/db/model utk smoke test end-to-end — sesuaikan kalau lain mesin.
_EXAMPLE_BAG    = '/mnt/ssd/Nasyih_MBKM/bags/mapping_20260623_154137'
_EXAMPLE_DB     = os.path.expanduser('~/.ros/rtabmap.db')
_EXAMPLE_ZONE   = os.path.join(
    ROOT, 'output', 'scene_zone_test', 'scene_zone_final_v9_final.yaml')
_EXAMPLE_MODEL  = os.path.join(ROOT, 'models', 'segformer_b0_ade')
HAS_E2E_DATA = (os.path.exists(_EXAMPLE_BAG) and os.path.isfile(_EXAMPLE_DB)
                and os.path.isfile(_EXAMPLE_ZONE) and os.path.isdir(_EXAMPLE_MODEL))


# ═══════════════════════════════════════════════════════════════════════════
# Helper: synthetic scene-zone grid (2 ruangan "workshop" terpisah wall/gap)
# ═══════════════════════════════════════════════════════════════════════════

def _img_to_world(img_row, img_col, height, resolution, origin_x=0.0, origin_y=0.0):
    row_from_bottom = height - 1 - img_row
    wx = origin_x + img_col * resolution
    wy = origin_y + row_from_bottom * resolution
    return wx, wy


def _make_synthetic_zone_yaml(tmpdir: str):
    """2 blob label='workshop'(6) di (rows 2:6, cols 2:6) dan (rows 2:6,
    cols 12:16), dipisah gap non-walkable (label 0) kolom 6..11.
    -> path ke yaml sidecar."""
    h, w, res = 10, 20, 0.1
    grid = np.zeros((h, w), dtype=np.uint8)
    grid[2:6, 2:6]   = 6  # workshop blob A
    grid[2:6, 12:16] = 6  # workshop blob B

    png_path = os.path.join(tmpdir, 'test_label.png')
    Image.fromarray(grid, mode='L').save(png_path)

    yaml_path = os.path.join(tmpdir, 'test_label.yaml')
    with open(yaml_path, 'w') as f:
        yaml.dump({
            'image': 'test_label.png',
            'resolution': res,
            'origin': [0.0, 0.0, 0],
            'width': w,
            'height': h,
            'label_map': {0: 'unknown', 1: 'corridor', 4: 'staircase',
                          5: 'wall', 6: 'workshop'},
        }, f)
    return yaml_path, h, w, res


# ═══════════════════════════════════════════════════════════════════════════
# 1. RoomZoneMap — sanity check murni numpy/scipy (no ROS/GPU)
# ═══════════════════════════════════════════════════════════════════════════

class TestRoomZoneMapSynthetic(unittest.TestCase):

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.yaml_path, self.h, self.w, self.res = _make_synthetic_zone_yaml(self.tmpdir)
        self.room_map = RoomZoneMap(self.yaml_path)

    def test_two_disconnected_rooms_detected(self):
        self.assertEqual(self.room_map.n_rooms, 2)

    def test_blobs_get_different_room_ids(self):
        wx_a, wy_a = _img_to_world(3, 3, self.h, self.res)
        wx_b, wy_b = _img_to_world(3, 13, self.h, self.res)
        rid_a = self.room_map.get_room_id(wx_a, wy_a)
        rid_b = self.room_map.get_room_id(wx_b, wy_b)
        self.assertGreater(rid_a, 0)
        self.assertGreater(rid_b, 0)
        self.assertNotEqual(rid_a, rid_b)

    def test_gap_point_is_zero(self):
        wx_gap, wy_gap = _img_to_world(3, 8, self.h, self.res)
        self.assertEqual(self.room_map.get_room_id(wx_gap, wy_gap), 0)

    def test_nearest_walkable_snaps_to_closest_room(self):
        wx_a, wy_a = _img_to_world(3, 3, self.h, self.res)
        rid_a = self.room_map.get_room_id(wx_a, wy_a)
        # 1px kanan dari tepi blob A (col 5) -> col 6, jauh dari blob B (col 12)
        wx_near_a, wy_near_a = _img_to_world(3, 6, self.h, self.res)
        self.assertEqual(self.room_map.get_room_id(wx_near_a, wy_near_a), 0)
        rid_snapped = self.room_map.get_room_id_nearest_walkable(
            wx_near_a, wy_near_a, search_radius_m=0.5)
        self.assertEqual(rid_snapped, rid_a)


# ═══════════════════════════════════════════════════════════════════════════
# 2. RoomHistogramAccumulator — murni numpy, pakai RoomZoneMap sintetis
# ═══════════════════════════════════════════════════════════════════════════

class TestRoomHistogramAccumulator(unittest.TestCase):

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.yaml_path, self.h, self.w, self.res = _make_synthetic_zone_yaml(self.tmpdir)
        self.room_map = RoomZoneMap(self.yaml_path)
        self.acc = RoomHistogramAccumulator(self.room_map, n_classes=8, snap_radius_m=0.3)
        self.wx_a, self.wy_a = _img_to_world(3, 3, self.h, self.res)
        self.wx_b, self.wy_b = _img_to_world(3, 13, self.h, self.res)

    def test_ratios_sum_to_one_per_room(self):
        room_a = self.acc.add_observation(self.wx_a, self.wy_a, 2)  # floor, obs 1
        self.acc.add_observation(self.wx_a, self.wy_a, 2)  # obs 2
        self.acc.add_observation(self.wx_a, self.wy_a, 2)  # obs 3
        self.acc.add_observation(self.wx_a, self.wy_a, 1)  # wall, obs 4
        ratios = self.acc.ratios()
        self.assertAlmostEqual(sum(ratios[room_a].values()), 1.0, places=6)
        self.assertAlmostEqual(ratios[room_a][2], 3 / 4, places=6)  # 3 floor dari 4 total

    def test_two_rooms_kept_separate(self):
        self.acc.add_observation(self.wx_a, self.wy_a, 2)
        self.acc.add_observation(self.wx_b, self.wy_b, 4)  # corridor
        ratios = self.acc.ratios()
        self.assertEqual(len(ratios), 2)

    def test_zero_label_dropped(self):
        room_id = self.acc.add_observation(self.wx_a, self.wy_a, 0)
        self.assertEqual(room_id, 0)
        self.assertEqual(self.acc.n_added, 0)

    def test_unreachable_point_dropped(self):
        acc = RoomHistogramAccumulator(self.room_map, n_classes=8, snap_radius_m=0.2)
        wx_far, wy_far = _img_to_world(3, 8, self.h, self.res)  # tengah gap
        room_id = acc.add_observation(wx_far, wy_far, 2)
        self.assertEqual(room_id, 0)
        self.assertEqual(acc.n_dropped_no_room, 1)
        self.assertEqual(acc.n_added, 0)

    def test_mark_frame_rooms_increments_frame_count(self):
        room_a = self.acc.add_observation(self.wx_a, self.wy_a, 2)
        self.acc.mark_frame_rooms({room_a})
        self.acc.mark_frame_rooms({room_a})
        self.assertEqual(self.acc.frame_counts[room_a], 2)


# ═══════════════════════════════════════════════════════════════════════════
# 3. fuse_room_tag/fuse_all_rooms — fusi rule-based, TIDAK butuh sklearn/training
# ═══════════════════════════════════════════════════════════════════════════

class TestFuseRoomTag(unittest.TestCase):

    def test_staircase_via_yolo_object(self):
        self.assertEqual(fuse_room_tag({'has_stairs': True, 'ratio_floor': 0.9}),
                          'staircase')

    def test_staircase_via_segformer_ratio(self):
        self.assertEqual(fuse_room_tag({'ratio_stairs': 0.10, 'ratio_floor': 0.5}),
                          'staircase')

    def test_workshop_via_lab_object(self):
        self.assertEqual(fuse_room_tag({'has_shelf': True, 'ratio_floor': 0.9}),
                          'workshop')

    def test_doorway_via_segformer_ratio(self):
        self.assertEqual(fuse_room_tag({'ratio_door': 0.10, 'ratio_floor': 0.1}),
                          'doorway')

    def test_corridor_via_floor_and_wall_ratio(self):
        self.assertEqual(
            fuse_room_tag({'ratio_floor': 0.5, 'ratio_wall': 0.3}), 'corridor')

    def test_unknown_fallback(self):
        self.assertEqual(fuse_room_tag({'ratio_ceiling': 0.9}), 'unknown')

    def test_stairs_priority_over_lab_objects(self):
        """Stairs harus menang walau ada objek lab juga di ruangan yg sama."""
        self.assertEqual(
            fuse_room_tag({'has_stairs': True, 'has_shelf': True}), 'staircase')

    def test_fuse_all_rooms_keys_match_input(self):
        features = {1: {'ratio_stairs': 0.1}, 2: {'has_shelf': True}}
        fused = fuse_all_rooms(features)
        self.assertEqual(set(fused), {1, 2})
        self.assertEqual(fused[1], 'staircase')
        self.assertEqual(fused[2], 'workshop')


# ═══════════════════════════════════════════════════════════════════════════
# 4. room_type_classifier LOOCV wiring — data sintetis, mudah dipisahkan
# ═══════════════════════════════════════════════════════════════════════════

class TestLOOCVWiring(unittest.TestCase):

    def setUp(self):
        # 8 ruangan sintetis, 2 kelas jelas terpisah lewat ratio_floor/ratio_wall
        self.features = {
            1: {'ratio_wall': 0.10, 'ratio_floor': 0.90, 'has_shelf': True},
            2: {'ratio_wall': 0.12, 'ratio_floor': 0.85, 'has_shelf': True},
            3: {'ratio_wall': 0.08, 'ratio_floor': 0.95, 'has_shelf': False},
            4: {'ratio_wall': 0.80, 'ratio_floor': 0.10},
            5: {'ratio_wall': 0.75, 'ratio_floor': 0.15},
            6: {'ratio_wall': 0.85, 'ratio_floor': 0.05},
            7: {'ratio_wall': 0.11, 'ratio_floor': 0.90, 'has_shelf': True},
            8: {'ratio_wall': 0.82, 'ratio_floor': 0.08},
        }
        self.labels = {
            rid: ('workshop' if rec['ratio_floor'] > 0.5 else 'corridor')
            for rid, rec in self.features.items()
        }

    @unittest.skipUnless(HAS_SKLEARN, "scikit-learn not installed")
    def test_combined_config_high_accuracy_on_separable_data(self):
        X, y, room_ids, cols = build_feature_matrix(self.features, self.labels, 'combined')
        self.assertEqual(len(room_ids), 8)
        acc, cm, label_order, y_pred = run_loocv(X, y, LogisticRegression(max_iter=1000))
        self.assertGreaterEqual(
            acc, 0.8,
            f"LOOCV accuracy {acc:.2f} rendah utk data yg seharusnya mudah dipisahkan "
            "— indikasi bug fold/label alignment, bukan soal kualitas data.")

    @unittest.skipUnless(HAS_SKLEARN, "scikit-learn not installed")
    def test_segformer_only_differs_from_yolo_only(self):
        """Sanity: 2 config fitur beda harus hasilkan matriks beda (bukan bug wiring
        yg diam-diam pakai kolom yg sama)."""
        X_seg, _, _, cols_seg   = build_feature_matrix(self.features, self.labels, 'segformer')
        X_yolo, _, _, cols_yolo = build_feature_matrix(self.features, self.labels, 'yolo')
        self.assertNotEqual(set(cols_seg), set(cols_yolo))
        self.assertNotEqual(X_seg.shape[1], 0)
        self.assertNotEqual(X_yolo.shape[1], 0)


# ═══════════════════════════════════════════════════════════════════════════
# 5. End-to-end smoke test — GPU/bag/db-gated
# ═══════════════════════════════════════════════════════════════════════════

class TestEndToEndSmoke(unittest.TestCase):

    @unittest.skipUnless(HAS_TORCH, "torch not installed")
    @unittest.skipUnless(HAS_E2E_DATA,
                          f"bag/db/zone-yaml/model contoh tidak ditemukan "
                          f"({_EXAMPLE_BAG}, {_EXAMPLE_DB}, {_EXAMPLE_ZONE}, {_EXAMPLE_MODEL}) "
                          "— test ini cuma jalan penuh di mesin yg punya data asli (Orin)")
    def test_run_room_features_from_db_smoke(self):
        from room_feature_extractor import run_room_features_from_db
        import rclpy
        out_path = os.path.join(tempfile.mkdtemp(), 'room_features_smoke.yaml')
        rclpy.init()
        try:
            records = run_room_features_from_db(
                _EXAMPLE_BAG, _EXAMPLE_DB, _EXAMPLE_ZONE, _EXAMPLE_MODEL, out_path,
                every_n=50)  # sangat jarang -- ini cuma smoke test, bukan full run
        finally:
            rclpy.shutdown()
        self.assertTrue(os.path.isfile(out_path))
        self.assertGreater(len(records), 0)
        room_map = RoomZoneMap(_EXAMPLE_ZONE)
        valid_ids = set(int(r) for r in np.unique(room_map.room_grid) if r > 0)
        for room_id in records:
            self.assertIn(room_id, valid_ids)


if __name__ == '__main__':
    print("=" * 60)
    print("  test_room_feature_extractor.py")
    print(f"  scikit-learn : {'OK' if HAS_SKLEARN else 'NOT INSTALLED (skip LOOCV tests)'}")
    print(f"  torch        : {'OK' if HAS_TORCH else 'NOT INSTALLED (skip e2e test)'}")
    print(f"  e2e data     : {'FOUND' if HAS_E2E_DATA else 'NOT FOUND (skip e2e test)'}")
    print("=" * 60)
    unittest.main(verbosity=2)
