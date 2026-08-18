#!/usr/bin/env python3
"""
test_semantic_exporter.py — Unit tests untuk semantic_map_exporter.py

Test tanpa ROS (mock semua ROS2 modules).
Fokus: logic centroid, label, YAML output, filtering.
"""

import sys
import os
import csv
import yaml
import struct
import tempfile
import unittest
from unittest.mock import MagicMock, patch
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ── Mock semua ROS2 modules sebelum import ────────────────────────────────
_ROS_MOCKS = [
    'rclpy', 'rclpy.node', 'rclpy.qos', 'rclpy.duration',
    'sensor_msgs', 'sensor_msgs.msg',
    'geometry_msgs', 'geometry_msgs.msg',
    'tf2_ros',
]
for _m in _ROS_MOCKS:
    sys.modules[_m] = MagicMock()

# Mock Node base class
_mock_node_cls = MagicMock()
_mock_node_cls.return_value = MagicMock()
sys.modules['rclpy.node'].Node = _mock_node_cls

sys.path.insert(0, os.path.join(ROOT, 'semantic'))

# Import hanya fungsi helper — bukan full Node (avoid ROS init)
# [E2] _find_reid_candidate pindah ke reid_core.py — diuji di
# tests/test_reid_core.py, bukan di sini lagi.
from semantic_map_exporter import (   # noqa: E402
    decode_pointcloud2_xyz_rgb, _load_class_names,
    _group_points_by_detection, _open_session_csv, _new_session_id,
    _sha256_file, _run_cmd, _check_disk_space,
)


# ────────────────────────────────────────────────────────────────────────────
# Helpers
# ────────────────────────────────────────────────────────────────────────────

def _pack_rgb(r: int, g: int, b: int) -> float:
    """Pack R,G,B → float32 dengan cara yang sama seperti semantic_projector."""
    packed = (r << 16) | (g << 8) | b
    return struct.unpack('<f', struct.pack('<I', packed))[0]


def _make_pointcloud2_msg(points):
    """
    Buat mock PointCloud2 msg dari list of (x, y, z, class_id, inst_id).
    """
    import sensor_msgs.msg as _sm  # already mocked
    msg = MagicMock()

    fields = {}
    for name, offset in [('x', 0), ('y', 4), ('z', 8), ('rgb', 12)]:
        f = MagicMock()
        f.name = name
        f.offset = offset
        fields[name] = f
    msg.fields = list(fields.values())

    point_step = 16  # 4 floats × 4 bytes
    n = len(points)
    msg.point_step = point_step
    msg.width = n
    msg.height = 1

    data = bytearray(n * point_step)
    for i, (x, y, z, cid, iid) in enumerate(points):
        base = i * point_step
        struct.pack_into('<f', data, base + 0, x)
        struct.pack_into('<f', data, base + 4, y)
        struct.pack_into('<f', data, base + 8, z)
        rgb_f = _pack_rgb(cid, iid, 0)
        struct.pack_into('<f', data, base + 12, rgb_f)
    msg.data = bytes(data)
    return msg


# ────────────────────────────────────────────────────────────────────────────
# Tests
# ────────────────────────────────────────────────────────────────────────────

class TestDecodePointcloud2(unittest.TestCase):

    def test_decode_basic(self):
        """Decode 3 titik → xyz dan rgb benar."""
        pts = [
            (1.0, 2.0, 0.0, 5, 3),   # class_id=5, inst_id=3
            (3.0, 4.0, 0.0, 2, 7),
            (5.0, 6.0, 0.0, 1, 1),
        ]
        msg = _make_pointcloud2_msg(pts)
        xyz, rgb_u32 = decode_pointcloud2_xyz_rgb(msg)

        self.assertIsNotNone(xyz)
        self.assertEqual(xyz.shape, (3, 3))
        np.testing.assert_allclose(xyz[:, 0], [1.0, 3.0, 5.0], atol=1e-4)
        np.testing.assert_allclose(xyz[:, 1], [2.0, 4.0, 6.0], atol=1e-4)

        # Check class_id dan instance_id dari rgb
        class_ids = ((rgb_u32 >> 16) & 0xFF)
        inst_ids  = ((rgb_u32 >> 8)  & 0xFF)
        np.testing.assert_array_equal(class_ids, [5, 2, 1])
        np.testing.assert_array_equal(inst_ids,  [3, 7, 1])

    def test_decode_empty(self):
        """Pointcloud kosong (0 poin) → return None."""
        msg = MagicMock()
        msg.fields = []
        msg.width  = 0
        msg.height = 0
        xyz, rgb = decode_pointcloud2_xyz_rgb(msg)
        self.assertIsNone(xyz)
        self.assertIsNone(rgb)

    def test_decode_missing_field(self):
        """Jika field 'rgb' tidak ada → return None."""
        msg = _make_pointcloud2_msg([(1.0, 2.0, 0.0, 1, 1)])
        # Hapus field rgb
        msg.fields = [f for f in msg.fields if f.name != 'rgb']
        xyz, rgb = decode_pointcloud2_xyz_rgb(msg)
        self.assertIsNone(xyz)


class TestLoadClassNames(unittest.TestCase):

    def test_empty_model_path(self):
        """model_path kosong → return empty dict."""
        result = _load_class_names('')
        self.assertEqual(result, {})

    def test_invalid_model_path(self):
        """model_path invalid → return empty dict (no crash)."""
        result = _load_class_names('/path/tidak/ada/model.pt')
        self.assertEqual(result, {})


class TestLabelConstruction(unittest.TestCase):
    """Test label construction logic (tanpa instantiate Node)."""

    def _make_label(self, class_names: dict, class_id: int, instance_id: int) -> str:
        """Replicate logika _make_label dari SemanticMapExporter."""
        name = class_names.get(class_id, f'class{class_id}')
        return f'{name}_{instance_id}'

    def test_label_with_names(self):
        names = {0: 'chair', 1: 'table', 2: 'person'}
        self.assertEqual(self._make_label(names, 0, 3), 'chair_3')
        self.assertEqual(self._make_label(names, 2, 7), 'person_7')

    def test_label_without_names(self):
        self.assertEqual(self._make_label({}, 5, 2), 'class5_2')

    def test_label_unknown_class(self):
        names = {0: 'chair'}
        self.assertEqual(self._make_label(names, 99, 1), 'class99_1')


class TestExportYamlFormat(unittest.TestCase):
    """Test bahwa output YAML kompatibel dengan object_goal_nav.py."""

    def _simulate_export(self, instances: dict, min_points: int,
                         class_names: dict) -> dict:
        """
        Replicate logika _export_yaml dari SemanticMapExporter.
        instances: {iid → {'class_id': int, 'xs': list, 'ys': list}}
        """
        out = {}
        for iid, entry in instances.items():
            if len(entry['xs']) < min_points:
                continue
            cid = entry['class_id']
            name = class_names.get(cid, f'class{cid}')
            label = f'{name}_{iid}'
            out[label] = {
                'x':          round(float(np.mean(entry['xs'])), 4),
                'y':          round(float(np.mean(entry['ys'])), 4),
                'class_id':   cid,
                'cell_count': len(entry['xs']),
            }
        return out

    def test_output_has_required_fields(self):
        """Setiap entry YAML harus punya x, y, class_id, cell_count."""
        instances = {
            3: {'class_id': 0, 'xs': [1.0] * 25, 'ys': [2.0] * 25},
        }
        result = self._simulate_export(instances, min_points=20,
                                       class_names={0: 'chair'})
        self.assertIn('chair_3', result)
        entry = result['chair_3']
        self.assertIn('x', entry)
        self.assertIn('y', entry)
        self.assertIn('class_id', entry)
        self.assertIn('cell_count', entry)

    def test_centroid_correct(self):
        """Centroid = rata-rata x dan y."""
        instances = {
            1: {'class_id': 1, 'xs': [0.0, 2.0, 4.0] * 10,
                'ys': [1.0, 3.0, 5.0] * 10},
        }
        result = self._simulate_export(instances, min_points=20,
                                       class_names={1: 'table'})
        self.assertIn('table_1', result)
        self.assertAlmostEqual(result['table_1']['x'], 2.0, places=3)
        self.assertAlmostEqual(result['table_1']['y'], 3.0, places=3)

    def test_min_points_filter(self):
        """Instance dengan poin < min_points tidak di-export."""
        instances = {
            1: {'class_id': 0, 'xs': [1.0] * 5,  'ys': [2.0] * 5},   # < 20
            2: {'class_id': 0, 'xs': [3.0] * 25, 'ys': [4.0] * 25},  # >= 20
        }
        result = self._simulate_export(instances, min_points=20,
                                       class_names={0: 'chair'})
        self.assertNotIn('chair_1', result)
        self.assertIn('chair_2', result)

    def test_yaml_round_trip(self):
        """YAML yang di-tulis bisa di-baca kembali dengan benar."""
        instances = {
            2: {'class_id': 0, 'xs': [1.5] * 30, 'ys': [2.5] * 30},
        }
        result = self._simulate_export(instances, min_points=20,
                                       class_names={0: 'chair'})

        with tempfile.NamedTemporaryFile(suffix='.yaml', mode='w',
                                        delete=False) as f:
            yaml.dump(result, f)
            fname = f.name

        try:
            with open(fname) as f:
                loaded = yaml.safe_load(f)
            self.assertIn('chair_2', loaded)
            self.assertAlmostEqual(loaded['chair_2']['x'], 1.5, places=3)
            self.assertAlmostEqual(loaded['chair_2']['y'], 2.5, places=3)
            self.assertEqual(loaded['chair_2']['class_id'], 0)
            self.assertEqual(loaded['chair_2']['cell_count'], 30)
        finally:
            os.unlink(fname)

    def test_yaml_compatible_with_object_goal_nav(self):
        """
        Simulasi load seperti object_goal_nav.py:
          obj = semantic_map[label]
          x, y = obj['x'], obj['y']
          obj['cell_count']
        """
        instances = {
            5: {'class_id': 2, 'xs': [3.7] * 40, 'ys': [-1.2] * 40},
        }
        result = self._simulate_export(instances, min_points=20,
                                       class_names={2: 'refrigerator'})

        label = 'refrigerator_5'
        self.assertIn(label, result)
        obj = result[label]
        x, y = obj['x'], obj['y']  # seperti object_goal_nav.py
        self.assertAlmostEqual(x, 3.7, places=3)
        self.assertAlmostEqual(y, -1.2, places=3)
        _ = obj['cell_count']    # harus tidak raise KeyError


class TestGroupPointsByDetection(unittest.TestCase):
    """[logger] Test grouping titik per (class_id, tracker_id) dalam 1 frame."""

    def test_groups_correctly(self):
        xs    = np.array([1.0, 1.2, 5.0])
        ys    = np.array([2.0, 2.2, 6.0])
        zs    = np.array([0.1, 0.1, 0.2])
        cids  = np.array([0, 0, 1])
        iids  = np.array([1, 1, 1])
        confs = np.array([0.8, 0.6, 0.9])
        out = _group_points_by_detection(xs, ys, zs, cids, iids, confs)

        self.assertEqual(len(out), 2)
        d0 = next(d for d in out if d['class_id'] == 0)
        d1 = next(d for d in out if d['class_id'] == 1)
        self.assertEqual(d0['n_points'], 2)
        self.assertAlmostEqual(d0['x'], 1.1, places=3)
        self.assertAlmostEqual(d0['y'], 2.1, places=3)
        self.assertEqual(d0['tracker_id'], 1)
        self.assertAlmostEqual(d0['confidence'], 0.7, places=3)   # mean(0.8, 0.6)
        self.assertEqual(d1['n_points'], 1)
        self.assertAlmostEqual(d1['x'], 5.0, places=3)
        self.assertAlmostEqual(d1['confidence'], 0.9, places=3)

    def test_empty_input(self):
        out = _group_points_by_detection(
            np.array([]), np.array([]), np.array([]),
            np.array([], dtype=int), np.array([], dtype=int), np.array([]))
        self.assertEqual(out, [])

    def test_distinguishes_same_class_different_tracker(self):
        """Dua tracker_id berbeda dalam kelas sama → dua baris terpisah."""
        xs, ys, zs = np.array([1.0, 9.0]), np.array([1.0, 9.0]), np.array([0.0, 0.0])
        cids, iids = np.array([0, 0]), np.array([1, 2])
        confs = np.array([0.5, 0.5])
        out = _group_points_by_detection(xs, ys, zs, cids, iids, confs)
        self.assertEqual(len(out), 2)
        self.assertEqual({d['tracker_id'] for d in out}, {1, 2})


class TestSessionId(unittest.TestCase):
    """[C1] Format session_id: S<YYYYMMDD>_<HHMM>."""

    def test_format(self):
        sid = _new_session_id()
        self.assertRegex(sid, r'^S\d{8}_\d{4}$')


class TestOpenSessionCsv(unittest.TestCase):
    """[A4] Satu file CSV per sesi, tidak pernah menimpa sesi lama."""

    def test_creates_file_with_header(self):
        with tempfile.TemporaryDirectory() as d:
            path, f, writer = _open_session_csv(d, 'det_S1.csv', ['a', 'b', 'c'])
            f.close()
            self.assertTrue(os.path.exists(path))
            with open(path) as fh:
                self.assertEqual(fh.readline().strip(), 'a,b,c')

    def test_never_overwrites_existing_session_file(self):
        """[checklist I] Kalau nama file sudah ada, jangan timpa — pakai sufiks."""
        with tempfile.TemporaryDirectory() as d:
            path1, f1, _ = _open_session_csv(d, 'det_S1.csv', ['a'])
            f1.write('sentinel-data-sesi-lama\n')
            f1.close()

            path2, f2, _ = _open_session_csv(d, 'det_S1.csv', ['a'])
            f2.close()

            self.assertNotEqual(path1, path2)
            with open(path1) as fh:
                content = fh.read()
            self.assertIn('sentinel-data-sesi-lama', content,
                'file sesi lama tidak boleh tertimpa oleh sesi baru dengan nama sama')

    def test_creates_directory_if_missing(self):
        with tempfile.TemporaryDirectory() as d:
            nested = os.path.join(d, 'nested', 'logs')
            path, f, _ = _open_session_csv(nested, 'det_S1.csv', ['a'])
            f.close()
            self.assertTrue(os.path.exists(path))


class TestH4CrashRecovery(unittest.TestCase):
    """
    [Uji Terima H4] Matikan paksa node di tengah sesi -> file log harus
    terbaca utuh sampai baris terakhir yang sempat di-flush, tidak korup.

    Simulasi "kill -9": tulis + flush() beberapa baris, JANGAN panggil
    close() (persis situasi proses dibunuh paksa sebelum sempat cleanup),
    lalu baca file itu lewat file HANDLE TERPISAH (meniru proses lain/
    proses baru yang membaca file setelah crash).
    """

    def test_flushed_rows_survive_without_close(self):
        with tempfile.TemporaryDirectory() as d:
            path, f, writer = _open_session_csv(d, 'det_S1.csv', ['a', 'b', 'c'])
            for i in range(5):
                writer.writerow([i, i * 2, f'row{i}'])
                f.flush()   # persis pola _log_frame_detections/_log_reid_timing
            # SENGAJA tidak close() — simulasi proses dibunuh paksa di sini.

            with open(path) as fh:
                rows = list(csv.reader(fh))
            self.assertEqual(rows[0], ['a', 'b', 'c'])
            self.assertEqual(len(rows), 6)   # header + 5 baris data
            for i in range(5):
                self.assertEqual(rows[i + 1], [str(i), str(i * 2), f'row{i}'])

    def test_last_unflushed_write_may_be_lost_but_prior_rows_intact(self):
        """
        Baris yang di-write() TANPA sempat flush() sebelum crash boleh
        hilang (tidak dijamin) — tapi baris-baris SEBELUMNYA yang sudah
        di-flush harus tetap utuh, bukan ikut korup/truncated.
        """
        with tempfile.TemporaryDirectory() as d:
            path, f, writer = _open_session_csv(d, 'det_S1.csv', ['a'])
            writer.writerow(['sudah_flush_1']); f.flush()
            writer.writerow(['sudah_flush_2']); f.flush()
            writer.writerow(['belum_flush_saat_crash'])   # tidak di-flush

            with open(path) as fh:
                content = fh.read()
            self.assertIn('sudah_flush_1', content)
            self.assertIn('sudah_flush_2', content)
            # Baris yang sudah ter-flush tidak boleh terpotong/korup
            lines = [ln for ln in content.splitlines() if ln]
            self.assertEqual(lines[0], 'a')
            self.assertEqual(lines[1], 'sudah_flush_1')
            self.assertEqual(lines[2], 'sudah_flush_2')


class TestSha256File(unittest.TestCase):
    """[D2] Hash model — deteksi kalau model di-retrain tanpa disadari."""

    def test_known_content_hash(self):
        with tempfile.NamedTemporaryFile(mode='wb', delete=False) as f:
            f.write(b'hello world')
            path = f.name
        try:
            import hashlib
            expected = hashlib.sha256(b'hello world').hexdigest()
            self.assertEqual(_sha256_file(path), expected)
        finally:
            os.unlink(path)

    def test_missing_file_returns_none(self):
        self.assertIsNone(_sha256_file('/tidak/ada/file/ini.pt'))

    def test_empty_path_returns_none(self):
        self.assertIsNone(_sha256_file(''))

    def test_different_content_different_hash(self):
        """Kalau model di-retrain (isi file beda), hash HARUS beda — ini
        yang jadi jaring pengaman deteksi retrain tak disadari (D2)."""
        with tempfile.NamedTemporaryFile(mode='wb', delete=False) as f1:
            f1.write(b'model version A')
            path1 = f1.name
        with tempfile.NamedTemporaryFile(mode='wb', delete=False) as f2:
            f2.write(b'model version B')
            path2 = f2.name
        try:
            self.assertNotEqual(_sha256_file(path1), _sha256_file(path2))
        finally:
            os.unlink(path1)
            os.unlink(path2)


class TestRunCmd(unittest.TestCase):

    def test_successful_command(self):
        out = _run_cmd(['echo', 'hello'])
        self.assertEqual(out, 'hello')

    def test_nonexistent_command_returns_none(self):
        out = _run_cmd(['perintah_yang_pasti_tidak_ada_xyz123'])
        self.assertIsNone(out)

    def test_empty_stdout_returns_none(self):
        out = _run_cmd(['true'])
        self.assertIsNone(out)


class TestCheckDiskSpace(unittest.TestCase):
    """[I] Peringatan disk penuh saat startup sesi eksperimen."""

    def test_ok_when_above_threshold(self):
        with tempfile.TemporaryDirectory() as d:
            free_gb, ok = _check_disk_space(d, min_free_gb=0.0001)
            self.assertTrue(ok)
            self.assertGreater(free_gb, 0)

    def test_not_ok_when_threshold_absurdly_high(self):
        with tempfile.TemporaryDirectory() as d:
            free_gb, ok = _check_disk_space(d, min_free_gb=1e9)   # 1 exabyte, mustahil lolos
            self.assertFalse(ok)

    def test_creates_directory_if_missing(self):
        with tempfile.TemporaryDirectory() as d:
            nested = os.path.join(d, 'belum', 'ada')
            _check_disk_space(nested, min_free_gb=0.0001)
            self.assertTrue(os.path.exists(nested))


def main():
    print('test_semantic_exporter.py — unit tests semantic_map_exporter.py\n')
    loader = unittest.TestLoader()
    suite = unittest.TestSuite([
        loader.loadTestsFromTestCase(TestDecodePointcloud2),
        loader.loadTestsFromTestCase(TestLoadClassNames),
        loader.loadTestsFromTestCase(TestLabelConstruction),
        loader.loadTestsFromTestCase(TestExportYamlFormat),
        loader.loadTestsFromTestCase(TestGroupPointsByDetection),
        loader.loadTestsFromTestCase(TestSessionId),
        loader.loadTestsFromTestCase(TestOpenSessionCsv),
        loader.loadTestsFromTestCase(TestH4CrashRecovery),
        loader.loadTestsFromTestCase(TestSha256File),
        loader.loadTestsFromTestCase(TestRunCmd),
        loader.loadTestsFromTestCase(TestCheckDiskSpace),
    ])
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)


if __name__ == '__main__':
    main()
