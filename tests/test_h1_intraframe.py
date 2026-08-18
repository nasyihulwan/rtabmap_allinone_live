#!/usr/bin/env python3
"""
test_h1_intraframe.py — [Uji Terima H1, checklist]

"Suntikkan dua deteksi bertumpang-tindih tinggi dalam satu frame yang sama.
Kriteria lolos: keduanya tetap tercatat sebagai dua baris log terpisah
dengan det_idx berbeda. Tidak ada penggabungan yang terjadi akibat
urutan pemrosesan."

Menjalankan KODE PRODUKSI ASLI (bukan reimplementasi/tiruan logika):
  - _group_points_by_detection()            (semantic_map_exporter.py)
  - SemanticMapExporter._log_frame_detections()  (dipanggil sbg unbound
    method di atas stand-in ringan — bukan Node ROS penuh, supaya tidak
    perlu mock rclpy/tf2_ros utuh, tapi method yang jalan tetap kode asli)
  - ReidAccumulator.add_detection()/promote_ready() (reid_core.py) — sisi
    mekanisme re-id, dipanggil persis pola _cloud_cb: semua add_detection()
    dulu, promote_ready() sekali di akhir frame.

Dikerjakan di rtabmap_allinone_live — BUKAN ws/project/rtabmap_allinone
(repo lama terpisah, remote github.com/nasyihulwan/rtabmap_allinone.git,
tidak disentuh sama sekali oleh test ini).
"""

import sys
import os
import csv
import io
import types
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


class _FakeNode:
    """
    Base class pengganti rclpy.node.Node — SENGAJA class Python biasa,
    BUKAN MagicMock(). Kalau Node adalah instance MagicMock(), pernyataan
    'class SemanticMapExporter(Node):' membuat SemanticMapExporter ikut
    jadi objek Mock (bukan class asli), sehingga
    SemanticMapExporter._log_frame_detections tidak bisa diakses sebagai
    unbound method — persis error yang ditemukan saat menulis test ini.
    Dengan base class NYATA seperti ini, seluruh method di file produksi
    (termasuk _log_frame_detections) tetap kode asli yang utuh dan bisa
    dipanggil langsung.
    """
    def __init__(self, *args, **kwargs):
        pass


sys.modules['rclpy.node'].Node = _FakeNode

sys.path.insert(0, os.path.join(ROOT, 'semantic'))

from semantic_map_exporter import (   # noqa: E402
    SemanticMapExporter, _group_points_by_detection, POSITION_LOG_FIELDS,
)
from reid_core import ReidAccumulator   # noqa: E402


def _make_overlapping_detections():
    """
    Dua deteksi, class_id SAMA, tracker_id BEDA, posisi world-frame SANGAT
    berdekatan (mensimulasikan bbox IoU tinggi di citra asal sebelum
    diproyeksikan ke 3D) — dalam SATU pointcloud message/frame yang sama.

    Jarak antar-cluster cuma ~2cm, jauh di bawah merge_dist manapun yang
    realistis (proposal pakai 1.0-1.5m) — kalau ada bug urutan pemrosesan
    intra-frame, ini skenario paling rentan untuk memicunya.
    """
    n = 25
    xs = np.concatenate([np.full(n, 3.00), np.full(n, 3.02)])
    ys = np.concatenate([np.full(n, 3.00), np.full(n, 3.01)])
    zs = np.full(2 * n, 0.30)
    cids = np.zeros(2 * n, dtype=int)                        # class_id sama: 0
    iids = np.concatenate([np.full(n, 1), np.full(n, 2)])    # tracker_id BEDA
    confs = np.full(2 * n, 0.8)
    return xs, ys, zs, cids, iids, confs


class TestH1IntraFrameFalseMerge(unittest.TestCase):
    """Uji Terima H1 — checklist."""

    def test_logger_writes_two_separate_rows_with_distinct_det_idx(self):
        """[H1 — sisi logger] _log_frame_detections (kode produksi asli)
        harus menulis 2 baris terpisah, det_idx 0 dan 1, bukan tergabung."""
        xs, ys, zs, cids, iids, confs = _make_overlapping_detections()

        # Sama persis dengan _cloud_cb: kelompokkan titik jadi deteksi
        # SEBELUM merge/logger — pakai fungsi produksi, bukan tiruan.
        detections = _group_points_by_detection(xs, ys, zs, cids, iids, confs)
        self.assertEqual(len(detections), 2,
            'harus terbentuk 2 deteksi terpisah dari 2 tracker_id')

        # Stand-in ringan utk 'self' — hanya atribut yang DIPAKAI
        # _log_frame_detections. Method yang dipanggil di bawah tetap kode
        # produksi asli (dipanggil sbg unbound method), bukan Node ROS
        # penuh — menghindari harus mock rclpy/tf2_ros utuh.
        buf = io.StringIO()
        writer = csv.writer(buf)
        # [alur asli] Header ditulis oleh _open_session_csv() SEKALI saat
        # file dibuka, SEBELUM _log_frame_detections pernah dipanggil —
        # direplikasi di sini supaya urutan baris persis sama dengan produksi.
        writer.writerow(POSITION_LOG_FIELDS)

        fake_self = types.SimpleNamespace(
            _log_writer=writer,
            _log_file=buf,
            session_id='S_TEST_H1',
            frame_seq=1,
            class_names={0: 'cone'},
            _lookup_robot_pose=lambda stamp: (1.0, 0.5, 0.1),
            _latest_cov=None,
        )
        stamp = types.SimpleNamespace(sec=100, nanosec=500_000_000)

        # Panggil METHOD ASLI (bukan reimplementasi) sebagai unbound method.
        SemanticMapExporter._log_frame_detections(fake_self, detections, stamp)

        buf.seek(0)
        rows = list(csv.reader(buf))
        header, data_rows = rows[0], rows[1:]

        print('\n--- potongan log H1 (det_idx 0 & 1, det. berdekatan) ---')
        print(','.join(header))
        for r in data_rows:
            print(','.join(r))
        print('--- akhir potongan log ---\n')

        self.assertEqual(header, POSITION_LOG_FIELDS,
            'header CSV harus persis sama dengan POSITION_LOG_FIELDS produksi')
        self.assertEqual(len(data_rows), 2,
            'harus ada TEPAT 2 baris data -> tidak boleh tergabung jadi 1')

        det_idx_col     = POSITION_LOG_FIELDS.index('det_idx')
        tracker_id_col  = POSITION_LOG_FIELDS.index('tracker_id')
        class_name_col  = POSITION_LOG_FIELDS.index('class_name')

        det_idxs     = sorted(int(r[det_idx_col]) for r in data_rows)
        tracker_ids  = sorted(int(r[tracker_id_col]) for r in data_rows)
        class_names  = {r[class_name_col] for r in data_rows}

        self.assertEqual(det_idxs, [0, 1],
            'det_idx harus 0 dan 1, dua-duanya beda (checklist H1)')
        self.assertEqual(tracker_ids, [1, 2],
            'kedua tracker_id asli harus tetap muncul terpisah, tidak melebur')
        self.assertEqual(class_names, {'cone'},
            'class_name harus konsisten (keduanya class_id=0)')

    def test_accumulator_does_not_merge_due_to_processing_order(self):
        """
        [H1 — sisi mekanisme] "Tidak ada penggabungan yang terjadi akibat
        urutan pemrosesan." ReidAccumulator dipanggil PERSIS seperti
        _cloud_cb: add_detection() untuk SEMUA deteksi di frame dulu, baru
        promote_ready() SEKALI di akhir — bukan per-deteksi. Kalau ada bug
        urutan (mis. reid dicek sebelum semua deteksi frame masuk), dua
        deteksi baru berdekatan ini akan salah tergabung.
        """
        xs, ys, zs, cids, iids, confs = _make_overlapping_detections()
        detections = _group_points_by_detection(xs, ys, zs, cids, iids, confs)

        acc = ReidAccumulator(merge_dist=1.0, min_points=20, reid_enabled=True)
        for d in detections:
            acc.add_detection(d['class_id'], d['tracker_id'], d['x'], d['y'], d['n_points'])
        events = acc.promote_ready()

        print(f'\n--- ReidAccumulator setelah 1 frame: n_confirmed={acc.n_confirmed} ---')
        for key, ev in events.items():
            print(f'  {key}: merged={ev["merged"]} target={ev["target"]}')

        self.assertEqual(acc.n_confirmed, 2,
            'dua deteksi baru berdekatan di frame yang sama tidak boleh jadi 1 instance')
        for key, ev in events.items():
            self.assertFalse(ev['merged'],
                f'{key} tidak boleh merge — keduanya provisional di frame yang sama')


def main():
    print('test_h1_intraframe.py — Uji Terima H1 (intra-frame false merge)\n')
    loader = unittest.TestLoader()
    suite = loader.loadTestsFromTestCase(TestH1IntraFrameFalseMerge)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)


if __name__ == '__main__':
    main()
