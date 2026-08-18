#!/usr/bin/env python3
"""
test_position_log_analyzer.py — Unit tests untuk semantic/position_log_analyzer.py

Tidak butuh ROS sama sekali (skrip analisis murni offline).
"""

import sys
import os
import csv
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'semantic'))

from position_log_analyzer import (   # noqa: E402
    session_centroids, noise_floor_report, guardband_threshold,
    replay_session, replay_multi_session, evaluate_reid,
    load_reid_timing_log, timing_report, append_ground_truth,
    load_labeled_detections, sweep_threshold, write_sweep_csv,
    match_instances_to_ground_truth, script_git_commit,
)


class TestSessionCentroids(unittest.TestCase):

    def test_groups_by_session_and_class(self):
        rows = [
            {'session_id': 's1', 'class_name': 'cone', 'x': 1.0, 'y': 1.0, 'z': 0.0},
            {'session_id': 's1', 'class_name': 'cone', 'x': 1.2, 'y': 1.0, 'z': 0.0},
            {'session_id': 's2', 'class_name': 'cone', 'x': 3.0, 'y': 3.0, 'z': 0.0},
            {'session_id': 's1', 'class_name': 'chair', 'x': 9.0, 'y': 9.0, 'z': 0.0},
        ]
        c = session_centroids(rows, 'cone')
        self.assertEqual(set(c.keys()), {'s1', 's2'})
        self.assertAlmostEqual(c['s1']['x'], 1.1, places=3)
        self.assertEqual(c['s1']['n_rows'], 2)
        self.assertAlmostEqual(c['s2']['x'], 3.0, places=3)


class TestNoiseFloorReport(unittest.TestCase):

    def test_known_deviations(self):
        # Simetris di sekitar origin → grand mean = (0,0) persis,
        # deviasi tiap sesi = 0.1 m kecuali s4 = 0.2 m.
        centroids = {
            's1': {'x':  0.1, 'y': 0.0, 'z': 0.0, 'n_rows': 20},
            's2': {'x': -0.1, 'y': 0.0, 'z': 0.0, 'n_rows': 20},
            's3': {'x':  0.0, 'y': 0.1, 'z': 0.0, 'n_rows': 20},
            's4': {'x':  0.0, 'y': -0.2, 'z': 0.0, 'n_rows': 20},
        }
        report = noise_floor_report(centroids)
        self.assertEqual(report['n_sessions'], 4)
        self.assertAlmostEqual(report['grand_mean_x'], 0.0, places=6)
        self.assertAlmostEqual(report['grand_mean_y'], -0.025, places=6)
        # s4 (0,-0.2) terjauh dari grand mean (0,-0.025) → deviasi 0.175
        self.assertAlmostEqual(report['d_max'], 0.175, places=3)
        self.assertEqual(
            max(report['per_session_deviation'], key=report['per_session_deviation'].get), 's4')

    def test_requires_at_least_two_sessions(self):
        with self.assertRaises(ValueError):
            noise_floor_report({'s1': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'n_rows': 5}})


class TestGuardbandThreshold(unittest.TestCase):

    def test_formula(self):
        self.assertAlmostEqual(guardband_threshold(0.2, 0.1), 0.3, places=6)


def _row(session_id, frame_seq, class_id, tracker_id, x, y, n_points=1):
    """Helper: baris position_log.csv minimal untuk test replay."""
    return {'session_id': session_id, 'frame_seq': frame_seq,
            'class_id': class_id, 'tracker_id': tracker_id,
            'x': x, 'y': y, 'n_points': n_points}


class TestReplaySession(unittest.TestCase):
    """[E1/E2] replay_session/replay_multi_session — lewat ReidAccumulator
    yang SAMA dipakai live, bukan reimplementasi terpisah."""

    def test_single_tracker_accumulates_across_frames(self):
        rows = [_row('s1', 1, 0, 5, 3.0, 3.0, n_points=10),
                _row('s1', 2, 0, 5, 3.02, 2.98, n_points=10)]
        snap = replay_session(rows, merge_dist=1.0, min_points=15)
        self.assertEqual(len(snap), 1)
        entry = list(snap.values())[0]
        self.assertEqual(entry['cell_count'], 20)

    def test_intra_frame_two_trackers_dont_merge(self):
        """[H1 di level replay] Dua tracker_id berbeda dalam frame_seq SAMA
        tidak boleh saling merge, walau posisinya berdekatan."""
        rows = [_row('s1', 1, 0, 1, 3.0, 3.0, n_points=20),
                _row('s1', 1, 0, 2, 3.02, 3.0, n_points=20)]
        snap = replay_session(rows, merge_dist=1.0, min_points=20)
        self.assertEqual(len(snap), 2)

    def test_cross_frame_merges_within_threshold(self):
        rows = [_row('s1', 1, 0, 1, 3.0, 3.0, n_points=20),
                _row('s1', 2, 0, 2, 3.1, 3.0, n_points=20)]   # frame beda
        snap = replay_session(rows, merge_dist=1.0, min_points=20)
        self.assertEqual(len(snap), 1)

    def test_beyond_threshold_stays_separate(self):
        rows = [_row('s1', 1, 0, 1, 0.0, 0.0, n_points=20),
                _row('s1', 2, 0, 2, 5.0, 0.0, n_points=20)]
        snap = replay_session(rows, merge_dist=1.0, min_points=20)
        self.assertEqual(len(snap), 2)


class TestReplayMultiSession(unittest.TestCase):

    def test_cross_session_reid(self):
        """
        Inti persoalan penelitian: tracker_id reset tiap sesi (restart),
        tapi objek fisik sama -> harus tetap satu instance lintas sesi.
        """
        session1 = [_row('s1', f, 0, 1, 2.0, 2.0, n_points=20) for f in range(1, 3)]
        # sesi 2: tracker_id RESET jadi 1 lagi (device_id lain krn restart YOLO),
        # posisi hampir sama (noise relokalisasi kecil)
        session2 = [_row('s2', f, 0, 1, 2.05, 1.98, n_points=20) for f in range(1, 3)]

        snap = replay_multi_session([session1, session2], merge_dist=1.0, min_points=20)
        self.assertEqual(len(snap), 1, 'objek sama lintas sesi harus tetap 1 instance')

    def test_cross_session_different_object_stays_separate(self):
        session1 = [_row('s1', f, 0, 1, 0.0, 0.0, n_points=20) for f in range(1, 3)]
        session2 = [_row('s2', f, 0, 1, 10.0, 10.0, n_points=20) for f in range(1, 3)]
        snap = replay_multi_session([session1, session2], merge_dist=1.0, min_points=20)
        self.assertEqual(len(snap), 2)


class TestEvaluateReid(unittest.TestCase):

    def test_false_merge_scenario(self):
        """
        Dua cone fisik (A, B) berjarak 0.5m — di bawah threshold 1.0m.
        Skenario ini persis kasus 'pengujian kepadatan' Fase K2:
        threshold terlalu longgar → salah menggabungkan objek berbeda.
        """
        dets = [
            {'x': 0.0, 'y': 0.0, 'class_name': 'cone', 'object_uid': 'A'},
            {'x': 0.1, 'y': 0.0, 'class_name': 'cone', 'object_uid': 'A'},
            {'x': 0.5, 'y': 0.0, 'class_name': 'cone', 'object_uid': 'B'},
        ]
        result = evaluate_reid(dets, threshold=1.0)
        self.assertEqual(result['tp'], 1)   # pasangan (det0,det1): sama A, sistem merge → benar
        self.assertEqual(result['fp'], 2)   # (det0,det2) & (det1,det2): beda objek, sistem merge → salah
        self.assertEqual(result['fn'], 0)
        self.assertEqual(result['tn'], 0)
        self.assertAlmostEqual(result['false_merge_rate'], 1.0, places=3)
        self.assertAlmostEqual(result['recall'], 1.0, places=3)

    def test_correct_disambiguation_with_tight_threshold(self):
        """Threshold ketat (0.2m) → A dan B (berjarak 0.5m) TIDAK digabung."""
        dets = [
            {'x': 0.0, 'y': 0.0, 'class_name': 'cone', 'object_uid': 'A'},
            {'x': 0.05, 'y': 0.0, 'class_name': 'cone', 'object_uid': 'A'},
            {'x': 0.5, 'y': 0.0, 'class_name': 'cone', 'object_uid': 'B'},
        ]
        result = evaluate_reid(dets, threshold=0.2)
        self.assertEqual(result['tp'], 1)
        self.assertEqual(result['fp'], 0)
        self.assertEqual(result['tn'], 2)
        self.assertAlmostEqual(result['false_merge_rate'], 0.0, places=3)

    def test_different_class_pairs_excluded_from_confusion_matrix(self):
        dets = [
            {'x': 0.0, 'y': 0.0, 'class_name': 'cone', 'object_uid': 'A'},
            {'x': 0.0, 'y': 0.0, 'class_name': 'chair', 'object_uid': 'X'},
        ]
        result = evaluate_reid(dets, threshold=1.0)
        self.assertEqual(result['tp'] + result['fp'] + result['fn'] + result['tn'], 0)


class TestTimingReport(unittest.TestCase):

    def test_basic_stats(self):
        rows = [
            {'reid_ms': 1.0, 'n_checked': 0, 'n_confirmed_instances': 3},
            {'reid_ms': 2.0, 'n_checked': 1, 'n_confirmed_instances': 3},
            {'reid_ms': 3.0, 'n_checked': 0, 'n_confirmed_instances': 4},
        ]
        report = timing_report(rows)
        self.assertEqual(report['n_frames'], 3)
        self.assertAlmostEqual(report['mean_ms'], 2.0, places=4)
        self.assertAlmostEqual(report['max_ms'], 3.0, places=4)
        self.assertAlmostEqual(report['mean_n_checked'], 1 / 3, places=4)

    def test_empty_raises(self):
        with self.assertRaises(ValueError):
            timing_report([])

    def test_load_reid_timing_log_roundtrip(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'reid_timing_log.csv')
            with open(path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['session_id', 'frame_seq', 'stamp_wall',
                            'reid_ms', 'n_checked', 'n_confirmed_instances',
                            'enable_reid', 'merge_dist'])
                w.writerow(['s1', 1, '100.0', '0.5', 0, 2, True, 1.0])
                w.writerow(['s1', 2, '100.1', '1.5', 1, 3, True, 1.0])
            rows = load_reid_timing_log(path)
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]['frame_seq'], 1)
            self.assertAlmostEqual(rows[1]['reid_ms'], 1.5, places=4)
            report = timing_report(rows)
            self.assertAlmostEqual(report['mean_ms'], 1.0, places=4)


class TestAppendGroundTruth(unittest.TestCase):
    """[G2] object_uid = identitas fisik permanen (checklist G1), bukan
    tracker_id. measured_wall default ke waktu sekarang kalau tidak diisi."""

    def test_creates_file_with_header_then_appends(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'gt.csv')
            append_ground_truth(path, 's1', 'CONE_A', 'cone', 1.0, 2.0,
                                 gt_z=0.0, marker_id='M1', instrument='meteran', note='sesi1')
            append_ground_truth(path, 's2', 'CONE_A', 'cone', 1.05, 2.02,
                                 gt_z=0.0, marker_id='M1', instrument='meteran', note='sesi2')

            detections = load_labeled_detections(path)
            self.assertEqual(len(detections), 2)
            self.assertEqual(detections[0]['object_uid'], 'CONE_A')
            self.assertAlmostEqual(detections[1]['x'], 1.05, places=4)
            self.assertEqual(detections[0]['session_id'], 's1')

            with open(path) as f:
                header = f.readline().strip().split(',')
            self.assertEqual(header, ['session_id', 'object_uid', 'class_name',
                                       'gt_x', 'gt_y', 'gt_z', 'marker_id',
                                       'measured_wall', 'instrument', 'note'])

    def test_measured_wall_defaults_to_now(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'gt.csv')
            append_ground_truth(path, 's1', 'CONE_A', 'cone', 1.0, 2.0)
            detections = load_labeled_detections(path)
            self.assertTrue(detections[0]['measured_wall'])   # tidak kosong


class TestSweepThreshold(unittest.TestCase):
    """[E3] Loop T 0..2.0 step 0.05 -> 41 nilai."""

    def test_produces_41_values_by_default(self):
        dets = [
            {'x': 0.0, 'y': 0.0, 'class_name': 'cone', 'object_uid': 'A'},
            {'x': 0.1, 'y': 0.0, 'class_name': 'cone', 'object_uid': 'A'},
        ]
        results = sweep_threshold(dets)
        self.assertEqual(len(results), 41)
        self.assertAlmostEqual(results[0]['T'], 0.0, places=6)
        self.assertAlmostEqual(results[-1]['T'], 2.0, places=6)
        self.assertAlmostEqual(results[1]['T'], 0.05, places=6)

    def test_precision_recall_change_across_sweep(self):
        """T kecil -> tidak digabung (recall rendah); T besar -> digabung
        (kedua objek beda kelas jadi tidak relevan di sini, cek dgn 1 kelas)."""
        dets = [
            {'x': 0.0, 'y': 0.0, 'class_name': 'cone', 'object_uid': 'A'},
            {'x': 0.5, 'y': 0.0, 'class_name': 'cone', 'object_uid': 'A'},
        ]
        results = sweep_threshold(dets, t_min=0.0, t_max=1.0, t_step=0.5)
        low = next(r for r in results if abs(r['T'] - 0.0) < 1e-9)
        high = next(r for r in results if abs(r['T'] - 1.0) < 1e-9)
        self.assertEqual(low['n_instances'], 2)   # T=0 -> tidak pernah merge
        self.assertEqual(high['n_instances'], 1)  # T=1.0 > jarak 0.5 -> merge

    def test_write_sweep_csv(self):
        with tempfile.TemporaryDirectory() as d:
            out = os.path.join(d, 'sweep.csv')
            results = [{'T': 0.0, 'precision': 1.0, 'recall': 1.0,
                        'false_merge_rate': 0.0, 'n_instances': 2,
                        'tp': 0, 'fp': 0, 'fn': 0, 'tn': 0}]
            write_sweep_csv(results, out)
            with open(out) as f:
                rows = list(csv.DictReader(f))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]['n_instances'], '2')


class TestMatchInstancesToGroundTruth(unittest.TestCase):
    """[E4] Hungarian assignment — deterministik untuk kasus ambigu."""

    def test_simple_one_to_one_match(self):
        instances = {(0, 1): {'class_id': 0, 'x': 1.0, 'y': 1.0, 'cell_count': 20}}
        class_names = {0: 'cone'}
        gt = [{'object_uid': 'CONE_A', 'class_name': 'cone', 'x': 1.02, 'y': 1.0}]
        result = match_instances_to_ground_truth(instances, class_names, gt)
        self.assertEqual(len(result['matched']), 1)
        ik, uid, dist = result['matched'][0]
        self.assertEqual(uid, 'CONE_A')
        self.assertAlmostEqual(dist, 0.02, places=3)
        self.assertEqual(result['unmatched_instances'], [])
        self.assertEqual(result['unmatched_gt'], [])

    def test_ambiguous_case_resolved_by_hungarian(self):
        """
        Dua instance, dua GT, posisinya saling silang (instance 1 lebih
        dekat ke GT B, instance 2 lebih dekat ke GT A secara individual,
        tapi assignment optimal GLOBAL adalah pasangan lurus) — Hungarian
        harus pilih total jarak minimum, bukan greedy per-instance.
        """
        instances = {
            (0, 1): {'class_id': 0, 'x': 0.0, 'y': 0.0, 'cell_count': 20},
            (0, 2): {'class_id': 0, 'x': 10.0, 'y': 0.0, 'cell_count': 20},
        }
        class_names = {0: 'cone'}
        gt = [
            {'object_uid': 'A', 'class_name': 'cone', 'x': 0.1, 'y': 0.0},
            {'object_uid': 'B', 'class_name': 'cone', 'x': 10.1, 'y': 0.0},
        ]
        result = match_instances_to_ground_truth(instances, class_names, gt)
        pairs = {(ik, uid) for ik, uid, _ in result['matched']}
        self.assertEqual(pairs, {((0, 1), 'A'), ((0, 2), 'B')})

    def test_different_class_never_matched(self):
        instances = {(0, 1): {'class_id': 0, 'x': 0.0, 'y': 0.0, 'cell_count': 20}}
        class_names = {0: 'cone'}
        gt = [{'object_uid': 'CHAIR_1', 'class_name': 'chair', 'x': 0.0, 'y': 0.0}]
        result = match_instances_to_ground_truth(instances, class_names, gt)
        self.assertEqual(result['matched'], [])
        self.assertEqual(result['unmatched_instances'], [(0, 1)])
        self.assertEqual(result['unmatched_gt'], ['CHAIR_1'])

    def test_max_dist_excludes_far_pairs(self):
        instances = {(0, 1): {'class_id': 0, 'x': 0.0, 'y': 0.0, 'cell_count': 20}}
        class_names = {0: 'cone'}
        gt = [{'object_uid': 'CONE_A', 'class_name': 'cone', 'x': 5.0, 'y': 0.0}]
        result = match_instances_to_ground_truth(instances, class_names, gt, max_dist=1.0)
        self.assertEqual(result['matched'], [])
        self.assertEqual(result['unmatched_instances'], [(0, 1)])
        self.assertEqual(result['unmatched_gt'], ['CONE_A'])


class TestScriptGitCommit(unittest.TestCase):
    """[E5] Reproduktibilitas."""

    def test_returns_nonempty_string(self):
        result = script_git_commit()
        self.assertIsInstance(result, str)
        self.assertTrue(len(result) > 0)


def main():
    print('test_position_log_analyzer.py — unit tests\n')
    loader = unittest.TestLoader()
    suite = unittest.TestSuite([
        loader.loadTestsFromTestCase(TestSessionCentroids),
        loader.loadTestsFromTestCase(TestNoiseFloorReport),
        loader.loadTestsFromTestCase(TestGuardbandThreshold),
        loader.loadTestsFromTestCase(TestReplaySession),
        loader.loadTestsFromTestCase(TestReplayMultiSession),
        loader.loadTestsFromTestCase(TestEvaluateReid),
        loader.loadTestsFromTestCase(TestTimingReport),
        loader.loadTestsFromTestCase(TestAppendGroundTruth),
        loader.loadTestsFromTestCase(TestSweepThreshold),
        loader.loadTestsFromTestCase(TestMatchInstancesToGroundTruth),
        loader.loadTestsFromTestCase(TestScriptGitCommit),
    ])
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)


if __name__ == '__main__':
    main()
