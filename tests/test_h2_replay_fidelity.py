#!/usr/bin/env python3
"""
test_h2_replay_fidelity.py — [Uji Terima H2, checklist] "paling menentukan"

Kriteria lolos: jumlah instance, kelas, dan posisi hasil replay identik
dengan keluaran live, dalam toleransi 1 mm.

Tidak bisa menjalankan sesi ROS/robot sungguhan dari sini (tidak ada
hardware) — tapi E2 sudah menjamin live (semantic_map_exporter.py) dan
replay (position_log_analyzer.py) memanggil MODUL YANG SAMA (reid_core.py)
untuk gating & akumulasi. Test ini memvalidasi bagian yang TERSISA di luar
reid_core.py yang harus tetap konsisten: cara live mengelompokkan titik
pointcloud jadi deteksi (_group_points_by_detection, granularitas A2) HARUS
menghasilkan angka yang, kalau ditulis ke CSV lalu dibaca ulang
(load_position_log) dan direplay (replay_session/replay_multi_session),
menghasilkan instance akhir yang identik dengan menjalankan accumulator
yang sama langsung secara "live" di memori — TANPA melalui CSV sama sekali.

Kalau test ini gagal, ada informasi yang hilang antara live dan CSV
(kolom kurang, konversi tipe salah, dsb) — persis skenario yang
diperingatkan checklist H2.
"""

import sys
import os
import csv
import tempfile
import unittest
from unittest.mock import MagicMock
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

_ROS_MOCKS = [
    'rclpy', 'rclpy.node', 'rclpy.qos', 'rclpy.duration',
    'sensor_msgs', 'sensor_msgs.msg',
    'geometry_msgs', 'geometry_msgs.msg',
    'tf2_ros',
]
for _m in _ROS_MOCKS:
    sys.modules[_m] = MagicMock()
_mock_node_cls = MagicMock()
_mock_node_cls.return_value = MagicMock()
sys.modules['rclpy.node'].Node = _mock_node_cls

sys.path.insert(0, os.path.join(ROOT, 'semantic'))

from semantic_map_exporter import (          # noqa: E402
    _group_points_by_detection, POSITION_LOG_FIELDS,
)
from reid_core import ReidAccumulator        # noqa: E402
from position_log_analyzer import (          # noqa: E402
    load_position_log, replay_multi_session,
)


def _make_frame(cids, iids, xs, ys, zs, confs):
    """Bungkus array titik mentah satu 'pointcloud message' jadi numpy arrays,
    persis input _group_points_by_detection di _cloud_cb."""
    return (np.array(xs), np.array(ys), np.array(zs),
            np.array(cids), np.array(iids), np.array(confs))


def _write_frame_to_csv(writer, session_id, frame_seq, detections):
    """Replikasi _log_frame_detections — satu baris CSV per deteksi,
    kolom persis POSITION_LOG_FIELDS."""
    for det_idx, d in enumerate(detections):
        writer.writerow([
            session_id, frame_seq, det_idx,
            f'{frame_seq * 0.1:.6f}', f'2026-01-01T09:00:{frame_seq:02d}',
            d['class_id'], f"class{d['class_id']}", d['tracker_id'],
            round(d['x'], 4), round(d['y'], 4), round(d['z'], 4),
            d['n_points'], round(d['confidence'], 4),
            1.0, 0.5, 0.1,   # robot_x, robot_y, robot_yaw (dummy, sama tiap frame)
            '', '', '',      # robot_cov_* kosong
        ])


class TestH2ReplayFidelity(unittest.TestCase):

    def _run_scenario(self, sessions_raw_frames, merge_dist=1.0, min_points=15):
        """
        sessions_raw_frames: list of list of frame-tuple (dari _make_frame),
        satu list per sesi (tracker_id di-reset tiap sesi, seperti live
        sesungguhnya setelah restart).

        Returns (live_snapshot, replay_snapshot).
        """
        # ── Jalur LIVE: langsung ke ReidAccumulator, seed antar-sesi ──────
        live_acc = ReidAccumulator(merge_dist=merge_dist, min_points=min_points)
        for session_idx, frames in enumerate(sessions_raw_frames):
            for xs, ys, zs, cids, iids, confs in frames:
                detections = _group_points_by_detection(xs, ys, zs, cids, iids, confs)
                for d in detections:
                    live_acc.add_detection(d['class_id'], d['tracker_id'],
                                            d['x'], d['y'], d['n_points'])
                live_acc.promote_ready()
            if session_idx < len(sessions_raw_frames) - 1:
                snap = live_acc.snapshot()
                live_acc = ReidAccumulator(merge_dist=merge_dist, min_points=min_points)
                for key, entry in snap.items():
                    live_acc.seed_instance(entry['class_id'], entry['x'], entry['y'],
                                            entry['cell_count'])
        live_snapshot = live_acc.snapshot()

        # ── Jalur REPLAY: tulis CSV per sesi, baca ulang, replay_multi_session ──
        with tempfile.TemporaryDirectory() as d:
            session_rows_list = []
            for session_idx, frames in enumerate(sessions_raw_frames):
                path = os.path.join(d, f'det_s{session_idx}.csv')
                with open(path, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(POSITION_LOG_FIELDS)
                    for frame_seq, (xs, ys, zs, cids, iids, confs) in enumerate(frames, start=1):
                        detections = _group_points_by_detection(xs, ys, zs, cids, iids, confs)
                        _write_frame_to_csv(writer, f's{session_idx}', frame_seq, detections)
                session_rows_list.append(load_position_log(path))

            replay_snapshot = replay_multi_session(
                session_rows_list, merge_dist=merge_dist, min_points=min_points)

        return live_snapshot, replay_snapshot

    def _assert_snapshots_identical(self, live_snap, replay_snap, tol_m=0.001):
        self.assertEqual(len(live_snap), len(replay_snap),
            f'jumlah instance beda: live={len(live_snap)} replay={len(replay_snap)}')

        live_sorted = sorted(live_snap.values(), key=lambda e: (e['class_id'], e['x'], e['y']))
        replay_sorted = sorted(replay_snap.values(), key=lambda e: (e['class_id'], e['x'], e['y']))

        for live_e, replay_e in zip(live_sorted, replay_sorted):
            self.assertEqual(live_e['class_id'], replay_e['class_id'], 'kelas harus sama')
            self.assertEqual(live_e['cell_count'], replay_e['cell_count'],
                'jumlah titik penyusun harus sama')
            self.assertLess(abs(live_e['x'] - replay_e['x']), tol_m,
                f"posisi x beda > {tol_m}m: live={live_e['x']} replay={replay_e['x']}")
            self.assertLess(abs(live_e['y'] - replay_e['y']), tol_m,
                f"posisi y beda > {tol_m}m: live={live_e['y']} replay={replay_e['y']}")

    def test_single_session_single_object(self):
        frames = [
            _make_frame([0]*20, [1]*20, [2.0]*20, [1.0]*20, [0.3]*20, [0.8]*20),
            _make_frame([0]*20, [1]*20, [2.02]*20, [0.99]*20, [0.3]*20, [0.8]*20),
        ]
        live_snap, replay_snap = self._run_scenario([frames])
        self._assert_snapshots_identical(live_snap, replay_snap)

    def test_intra_frame_two_objects_no_false_merge(self):
        """[H1+H2 gabungan] Dua objek berdekatan dalam SATU frame yang sama
        (tracker_id berbeda) — replay harus tetap 2 instance seperti live."""
        frames = [
            _make_frame([0]*20 + [0]*20, [1]*20 + [2]*20,
                        [3.0]*20 + [3.05]*20, [3.0]*40, [0.3]*40, [0.8]*40),
            _make_frame([0]*20 + [0]*20, [1]*20 + [2]*20,
                        [3.01]*20 + [3.04]*20, [3.0]*40, [0.3]*40, [0.8]*40),
        ]
        live_snap, replay_snap = self._run_scenario([frames], merge_dist=1.0, min_points=15)
        self.assertEqual(len(live_snap), 2)
        self._assert_snapshots_identical(live_snap, replay_snap)

    def test_cross_session_reid_and_false_merge_disambiguation(self):
        """
        Skenario realistis Fase K1/K2: sesi 1 punya objek A confirmed.
        Sesi 2 (restart, tracker_id reset ke 1) punya deteksi objek A lagi
        (posisi dekat -> harus merge) DAN objek B baru (posisi jauh ->
        instance terpisah) dengan tracker_id yang SAMA (1 vs 2, reset).
        """
        session1 = [
            _make_frame([0]*20, [1]*20, [2.0]*20, [2.0]*20, [0.3]*20, [0.8]*20),
        ]
        session2 = [
            _make_frame([0]*20 + [0]*20, [1]*20 + [2]*20,
                        [2.03]*20 + [8.0]*20, [1.98]*20 + [8.0]*20,
                        [0.3]*40, [0.8]*40),
        ]
        live_snap, replay_snap = self._run_scenario([session1, session2],
                                                       merge_dist=1.0, min_points=15)
        self.assertEqual(len(live_snap), 2, 'objek A (merge) + objek B (baru) = 2 instance')
        self._assert_snapshots_identical(live_snap, replay_snap)

    def test_multi_frame_gradual_accumulation(self):
        """Deteksi tersebar di banyak frame kecil (bukan satu frame besar)
        sebelum mencapai min_points — replay harus tetap identik."""
        frames = [_make_frame([0]*5, [1]*5, [4.0 + i*0.001]*5, [4.0]*5, [0.3]*5, [0.8]*5)
                  for i in range(10)]   # 10 frame x 5 titik = 50 titik total
        live_snap, replay_snap = self._run_scenario([frames], merge_dist=1.0, min_points=20)
        self.assertEqual(len(live_snap), 1)
        self._assert_snapshots_identical(live_snap, replay_snap)


def main():
    print('test_h2_replay_fidelity.py — Uji Terima H2 (kesetiaan replay)\n')
    loader = unittest.TestLoader()
    suite = loader.loadTestsFromTestCase(TestH2ReplayFidelity)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)


if __name__ == '__main__':
    main()
