#!/usr/bin/env python3
"""
test_reid_core.py — Unit tests untuk semantic/reid_core.py

Modul ini TIDAK punya dependency ROS sama sekali — test jalan langsung
tanpa mock apa pun.
"""

import sys
import os
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'semantic'))

from reid_core import ReidAccumulator, find_reid_candidate   # noqa: E402


class TestFindReidCandidate(unittest.TestCase):

    def test_merge_within_threshold(self):
        instances = {(0, 1): {'class_id': 0, 'xs': [1.0] * 25, 'ys': [1.0] * 25}}
        best_key, dist = find_reid_candidate(
            1.3, 1.0, 0, instances, provisional_keys=set(),
            merge_dist=1.0, exclude_key=(0, 2))
        self.assertEqual(best_key, (0, 1))
        self.assertAlmostEqual(dist, 0.3, places=3)

    def test_no_merge_beyond_threshold(self):
        instances = {(0, 1): {'class_id': 0, 'xs': [1.0] * 25, 'ys': [1.0] * 25}}
        best_key, _ = find_reid_candidate(
            5.0, 5.0, 0, instances, provisional_keys=set(),
            merge_dist=1.0, exclude_key=(0, 2))
        self.assertIsNone(best_key)

    def test_reid_disabled_never_merges(self):
        instances = {(0, 1): {'class_id': 0, 'xs': [1.0] * 25, 'ys': [1.0] * 25}}
        best_key, _ = find_reid_candidate(
            1.0, 1.0, 0, instances, provisional_keys=set(),
            merge_dist=1.0, exclude_key=(0, 2), reid_enabled=False)
        self.assertIsNone(best_key)

    def test_no_merge_different_class(self):
        """Kandidat kelas berbeda tidak boleh jadi target merge, walau posisi sama persis."""
        instances = {(1, 1): {'class_id': 1, 'xs': [1.0] * 25, 'ys': [1.0] * 25}}
        best_key, _ = find_reid_candidate(
            1.0, 1.0, 0, instances, provisional_keys=set(),
            merge_dist=1.0, exclude_key=(0, 2))
        self.assertIsNone(best_key)

    def test_picks_nearest_among_multiple_candidates(self):
        instances = {
            (0, 1): {'class_id': 0, 'xs': [1.0] * 25, 'ys': [1.0] * 25},
            (0, 2): {'class_id': 0, 'xs': [1.08] * 25, 'ys': [1.08] * 25},
        }
        best_key, _ = find_reid_candidate(
            1.1, 1.1, 0, instances, provisional_keys=set(),
            merge_dist=1.0, exclude_key=(0, 9))
        self.assertEqual(best_key, (0, 2))


class TestReidAccumulatorBasic(unittest.TestCase):

    def test_new_detection_confirmed_when_no_candidate(self):
        acc = ReidAccumulator(merge_dist=1.0, min_points=5)
        for _ in range(5):
            acc.add_detection(0, 1, 3.0, 3.0, n_points=1)
        events = acc.promote_ready()
        self.assertEqual(len(events), 1)
        key = list(events.keys())[0]
        self.assertFalse(events[key]['merged'])
        self.assertEqual(acc.n_confirmed, 1)

    def test_cross_session_merge(self):
        """Instance lama confirmed, instance baru (tracker_id beda krn restart)
        berdekatan posisi -> harus merge ke instance lama."""
        acc = ReidAccumulator(merge_dist=1.0, min_points=5)
        acc.seed_instance(class_id=0, tracker_id=1, x=2.0, y=2.0, weight=30)  # confirmed

        for _ in range(5):
            acc.add_detection(0, 7, 2.1, 2.0, n_points=1)   # tracker_id baru, posisi dekat
        events = acc.promote_ready()

        key7 = (0, 7)
        self.assertTrue(events[key7]['merged'])
        self.assertEqual(events[key7]['target'], (0, 1))
        self.assertEqual(acc.n_confirmed, 1)
        self.assertNotIn((0, 7), acc.instances)   # sudah dipindah ke (0,1)

    def test_no_merge_beyond_threshold_end_to_end(self):
        acc = ReidAccumulator(merge_dist=0.5, min_points=5)
        acc.seed_instance(class_id=0, tracker_id=1, x=2.0, y=2.0, weight=30)

        for _ in range(5):
            acc.add_detection(0, 7, 5.0, 5.0, n_points=1)   # jauh
        events = acc.promote_ready()

        self.assertFalse(events[(0, 7)]['merged'])
        self.assertEqual(acc.n_confirmed, 2)

    def test_reid_enabled_false_baseline_naif(self):
        acc = ReidAccumulator(merge_dist=1.0, min_points=5, reid_enabled=False)
        acc.seed_instance(class_id=0, tracker_id=1, x=2.0, y=2.0, weight=30)
        for _ in range(5):
            acc.add_detection(0, 7, 2.01, 2.0, n_points=1)   # sangat dekat, tapi reid off
        events = acc.promote_ready()
        self.assertFalse(events[(0, 7)]['merged'])
        self.assertEqual(acc.n_confirmed, 2)


class TestIntraFrameNoFalseMerge(unittest.TestCase):

    def test_two_new_detections_same_frame_dont_merge(self):
        """
        [dummy test H1] Dua deteksi BARU (berdekatan) ditambahkan SEBELUM
        promote_ready() dipanggil (mensimulasikan satu frame yang sama)
        tidak boleh saling merge, walau posisinya sangat dekat dan cukup
        titik untuk lolos min_points begitu promote_ready() dipanggil.
        """
        acc = ReidAccumulator(merge_dist=1.0, min_points=5)
        for _ in range(5):
            acc.add_detection(0, 1, 3.0, 3.0, n_points=1)
        for _ in range(5):
            acc.add_detection(0, 2, 3.02, 3.0, n_points=1)   # 2cm — sangat dekat

        events = acc.promote_ready()   # dipanggil SEKALI, setelah kedua deteksi masuk

        self.assertFalse(events[(0, 1)]['merged'])
        self.assertFalse(events[(0, 2)]['merged'])
        self.assertEqual(acc.n_confirmed, 2)


class TestDeterministicOrdering(unittest.TestCase):

    def test_promote_order_is_sorted_not_set_order(self):
        """
        [D1] Buat skenario di mana urutan pemrosesan MEMENGARUHI hasil:
        key (0,1) merge ke instance confirmed X dan menggeser mean X;
        key (0,99) posisinya persis di ambang batas terhadap X — HANYA
        merge kalau diproses SETELAH (0,1) (karena mean X sudah bergeser
        mendekat). sorted() menjamin (0,1) diproses duluan dari (0,99).
        """
        acc = ReidAccumulator(merge_dist=0.5, min_points=3)
        acc.seed_instance(class_id=0, tracker_id=0, x=0.0, y=0.0, weight=10)  # X, confirmed

        for _ in range(3):
            acc.add_detection(0, 1, 0.3, 0.0, n_points=1)     # dekat X, akan merge duluan
        for _ in range(3):
            acc.add_detection(0, 99, 0.55, 0.0, n_points=1)   # di luar 0.5 dari X asli,
                                                                # tapi di dalam 0.5 dari X
                                                                # SETELAH bergeser oleh (0,1)

        events = acc.promote_ready()

        self.assertTrue(events[(0, 1)]['merged'])
        # Mean X setelah merge (0,1): weighted mean(0.0*10 + 0.3*3)/13 = 0.0692
        # dist((0,99)=0.55, X_baru=0.0692) = 0.4808 < 0.5 -> merge
        # dist((0,99)=0.55, X_lama=0.0)    = 0.55        > 0.5 -> tidak merge
        # Kalau urutan tidak deterministik/terbalik, hasil ini bisa berbeda.
        self.assertTrue(events[(0, 99)]['merged'],
            'urutan promote_ready harus sorted() — (0,1) diproses sebelum (0,99)')


class TestSeedAndSnapshot(unittest.TestCase):

    def test_seed_then_snapshot_roundtrip(self):
        acc = ReidAccumulator(merge_dist=1.0, min_points=20)
        acc.seed_instance(class_id=0, tracker_id=3, x=1.234, y=5.678, weight=42)
        snap = acc.snapshot()
        self.assertIn((0, 3), snap)
        self.assertAlmostEqual(snap[(0, 3)]['x'], 1.234, places=3)
        self.assertAlmostEqual(snap[(0, 3)]['y'], 5.678, places=3)
        self.assertEqual(snap[(0, 3)]['cell_count'], 42)

    def test_snapshot_filters_below_min_points(self):
        acc = ReidAccumulator(merge_dist=1.0, min_points=20)
        for _ in range(5):
            acc.add_detection(0, 1, 1.0, 1.0, n_points=1)
        snap = acc.snapshot()
        self.assertEqual(snap, {})   # belum lolos min_points, belum di-promote pula

    def test_default_seed_key_never_collides_with_live_tracker_id(self):
        """
        [regresi bug kritis] SimpleIoUTracker (yolo_detector.py) reset
        next_id ke 1 setiap node restart, jadi objek pertama tiap sesi
        baru HAMPIR PASTI dapat tracker_id=1 lagi. Kalau seed_instance()
        memakai tracker_id mentah dari sesi lama sebagai key, deteksi baru
        dengan tracker_id=1 akan collide langsung dan diam-diam TERTIMPA
        ke instance lama TANPA pernah melalui find_reid_candidate (jarak
        tidak pernah dicek). Default seed_instance (tanpa tracker_id
        eksplisit) harus menghasilkan key yang mustahil collide dengan
        tracker_id live asli (selalu positif, 1..254).
        """
        acc = ReidAccumulator(merge_dist=0.5, min_points=1)
        acc.seed_instance(class_id=0, x=0.0, y=0.0, weight=20)   # objek A, dari sesi lalu

        # Sesi baru: objek fisik BEDA (jauh), tapi tracker_id kebetulan = 1
        # lagi (restart tracker). Kalau seed key collide, ini akan salah
        # tertimpa ke objek A alih-alih dievaluasi lewat jarak.
        acc.add_detection(class_id=0, tracker_id=1, x=10.0, y=10.0, n_points=1)
        events = acc.promote_ready()

        self.assertEqual(acc.n_confirmed, 2,
            'objek beda posisi harus tetap 2 instance, bukan collide via key mentah')
        self.assertFalse(events[(0, 1)]['merged'])


class TestPerDetectionEquivalentToPerPoint(unittest.TestCase):
    """[A2/reid_core docstring] Verifikasi klaim kesetaraan matematis:
    add_detection per-titik (n_points=1 berkali-kali) vs satu panggilan
    per-deteksi dengan n_points=N harus menghasilkan mean yang identik."""

    def test_per_point_vs_per_detection_same_mean(self):
        pts = [(1.0, 2.0), (1.2, 2.2), (0.8, 1.9), (1.1, 2.1)]

        acc_per_point = ReidAccumulator(merge_dist=1.0, min_points=4)
        for x, y in pts:
            acc_per_point.add_detection(0, 1, x, y, n_points=1)

        mean_x = sum(p[0] for p in pts) / len(pts)
        mean_y = sum(p[1] for p in pts) / len(pts)
        acc_per_detection = ReidAccumulator(merge_dist=1.0, min_points=4)
        acc_per_detection.add_detection(0, 1, mean_x, mean_y, n_points=len(pts))

        snap_a = acc_per_point.snapshot()
        snap_b = acc_per_detection.snapshot()
        self.assertAlmostEqual(snap_a[(0, 1)]['x'], snap_b[(0, 1)]['x'], places=6)
        self.assertAlmostEqual(snap_a[(0, 1)]['y'], snap_b[(0, 1)]['y'], places=6)
        self.assertEqual(snap_a[(0, 1)]['cell_count'], snap_b[(0, 1)]['cell_count'])


def main():
    print('test_reid_core.py — unit tests\n')
    loader = unittest.TestLoader()
    suite = unittest.TestSuite([
        loader.loadTestsFromTestCase(TestFindReidCandidate),
        loader.loadTestsFromTestCase(TestReidAccumulatorBasic),
        loader.loadTestsFromTestCase(TestIntraFrameNoFalseMerge),
        loader.loadTestsFromTestCase(TestDeterministicOrdering),
        loader.loadTestsFromTestCase(TestSeedAndSnapshot),
        loader.loadTestsFromTestCase(TestPerDetectionEquivalentToPerPoint),
    ])
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)


if __name__ == '__main__':
    main()
