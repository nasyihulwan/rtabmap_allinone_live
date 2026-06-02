#!/usr/bin/env python3
"""
test_object_goal_nav.py — Unit tests untuk object_goal_nav.py

Test tanpa ROS (mock semua ROS2 modules).
Fokus: logic navigasi, boundary check, retry offset, YAML load.
"""

import sys
import os
import types
import tempfile
import yaml
import unittest
from unittest.mock import MagicMock
import math

ROOT        = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SEMANTIC_DIR = os.path.join(ROOT, 'semantic')

# ── Build proper fake Node class (rclpy.node.Node perlu jadi real class) ─
_FakeNode = type('Node', (), {
    '__init__':            lambda self, name: None,
    'get_logger':          lambda self: MagicMock(),
    'declare_parameter':   lambda self, *a, **kw: None,
    'get_parameter':       lambda self, *a: MagicMock(),
    'create_subscription': lambda self, *a, **kw: None,
    'create_publisher':    lambda self, *a, **kw: None,
    'create_timer':        lambda self, *a, **kw: None,
    'get_clock':           lambda self: MagicMock(),
})

_mock_rclpy_node = types.ModuleType('rclpy.node')
_mock_rclpy_node.Node = _FakeNode

# ── Mock semua ROS2 modules ───────────────────────────────────────────────
for _m in ['rclpy', 'rclpy.action', 'rclpy.qos',
           'std_msgs', 'std_msgs.msg',
           'geometry_msgs', 'geometry_msgs.msg',
           'nav2_msgs', 'nav2_msgs.action']:
    sys.modules[_m] = MagicMock()

sys.modules['rclpy.node'] = _mock_rclpy_node

sys.path.insert(0, SEMANTIC_DIR)
from object_goal_nav import ObjectGoalNav   # noqa: E402


# ────────────────────────────────────────────────────────────────────────────
# Helper: buat pseudo-self dengan atribut yang diperlukan helper methods
# ────────────────────────────────────────────────────────────────────────────

def _make_self(map_origin_x=-8.0, map_origin_y=-5.0,
               map_width=300, map_height=600, map_resolution=0.05,
               goal_tolerance=0.3, max_retry=5, retry_offset=0.5):
    """SimpleNamespace dengan atribut yang diperlukan ObjectGoalNav helpers."""
    ns = types.SimpleNamespace()
    ns.map_x_min    = map_origin_x
    ns.map_x_max    = map_origin_x + map_width * map_resolution
    ns.map_y_min    = map_origin_y
    ns.map_y_max    = map_origin_y + map_height * map_resolution
    ns.tolerance    = goal_tolerance
    ns.max_retry    = max_retry
    ns.retry_offset = retry_offset
    ns.get_logger   = lambda: MagicMock()
    return ns


# ────────────────────────────────────────────────────────────────────────────
# Tests: _is_in_map
# ────────────────────────────────────────────────────────────────────────────

class TestIsInMap(unittest.TestCase):
    def setUp(self):
        self.s = _make_self(
            map_origin_x=-8.0, map_origin_y=-5.0,
            map_width=300, map_height=600, map_resolution=0.05)
        # x_max = -8+300*0.05 = 7.0,  y_max = -5+600*0.05 = 25.0

    def _call(self, x, y):
        return ObjectGoalNav._is_in_map(self.s, x, y)

    def test_center_inside(self):
        self.assertTrue(self._call(0.0, 10.0))

    def test_on_boundary_origin(self):
        self.assertTrue(self._call(-8.0, -5.0))

    def test_on_boundary_far(self):
        self.assertTrue(self._call(7.0, 25.0))

    def test_outside_x_positive(self):
        self.assertFalse(self._call(10.0, 0.0))

    def test_outside_y_positive(self):
        self.assertFalse(self._call(0.0, 30.0))

    def test_outside_negative(self):
        self.assertFalse(self._call(-10.0, 0.0))

    def test_outside_y_negative(self):
        self.assertFalse(self._call(0.0, -10.0))


# ────────────────────────────────────────────────────────────────────────────
# Tests: _clamp_to_map
# ────────────────────────────────────────────────────────────────────────────

class TestClampToMap(unittest.TestCase):
    def setUp(self):
        # map x=[0,10], y=[0,10] → effective dengan margin 0.5: [0.5, 9.5]
        self.s = _make_self(
            map_origin_x=0.0, map_origin_y=0.0,
            map_width=200, map_height=200, map_resolution=0.05)

    def _call(self, x, y):
        return ObjectGoalNav._clamp_to_map(self.s, x, y)

    def test_inside_unchanged(self):
        cx, cy = self._call(5.0, 5.0)
        self.assertAlmostEqual(cx, 5.0)
        self.assertAlmostEqual(cy, 5.0)

    def test_clamp_x_max(self):
        cx, cy = self._call(15.0, 5.0)
        self.assertAlmostEqual(cx, 9.5)   # 10 - 0.5 margin

    def test_clamp_x_min(self):
        cx, cy = self._call(-5.0, 5.0)
        self.assertAlmostEqual(cx, 0.5)   # 0 + 0.5 margin

    def test_clamp_y_max(self):
        cx, cy = self._call(5.0, 50.0)
        self.assertAlmostEqual(cy, 9.5)

    def test_clamp_y_min(self):
        cx, cy = self._call(5.0, -50.0)
        self.assertAlmostEqual(cy, 0.5)

    def test_clamp_both(self):
        cx, cy = self._call(-100.0, 100.0)
        self.assertAlmostEqual(cx, 0.5)
        self.assertAlmostEqual(cy, 9.5)

    def test_on_effective_boundary_stays(self):
        cx, cy = self._call(0.5, 9.5)
        self.assertAlmostEqual(cx, 0.5)
        self.assertAlmostEqual(cy, 9.5)

    def test_result_always_inside_map(self):
        """Hasil clamp harus selalu di dalam map (inclusive)."""
        for x in [-50.0, 0.0, 5.0, 15.0]:
            for y in [-50.0, 0.0, 5.0, 15.0]:
                cx, cy = self._call(x, y)
                self.assertTrue(
                    ObjectGoalNav._is_in_map(self.s, cx, cy),
                    f'clamp({x},{y})=({cx},{cy}) masih di luar map!')


# ────────────────────────────────────────────────────────────────────────────
# Tests: _offset_toward_center
# ────────────────────────────────────────────────────────────────────────────

class TestOffsetTowardCenter(unittest.TestCase):
    def setUp(self):
        self.s = _make_self(retry_offset=0.5)

    def _call(self, x, y, step):
        return ObjectGoalNav._offset_toward_center(self.s, x, y, step)

    def test_step1_north(self):
        nx, ny = self._call(1.0, 2.0, 1)
        self.assertAlmostEqual(nx, 1.0)
        self.assertAlmostEqual(ny, 2.5)   # +0.5 arah Y positif (utara)

    def test_step2_south(self):
        nx, ny = self._call(1.0, 2.0, 2)
        self.assertAlmostEqual(nx, 1.0)
        self.assertAlmostEqual(ny, 1.5)   # -0.5 arah Y negatif (selatan)

    def test_step3_east(self):
        nx, ny = self._call(1.0, 2.0, 3)
        self.assertAlmostEqual(nx, 1.5)   # +0.5 arah X positif (timur)
        self.assertAlmostEqual(ny, 2.0)

    def test_step4_west(self):
        nx, ny = self._call(1.0, 2.0, 4)
        self.assertAlmostEqual(nx, 0.5)   # -0.5 arah X negatif (barat)
        self.assertAlmostEqual(ny, 2.0)

    def test_step5_far_north(self):
        nx, ny = self._call(1.0, 2.0, 5)
        self.assertAlmostEqual(nx, 1.0)
        self.assertAlmostEqual(ny, 3.0)   # +1.0 (2 × retry_offset)

    def test_all_steps_produce_nonzero_offset(self):
        """Semua step 1-5 harus menghasilkan offset dari posisi asal."""
        for step in range(1, 6):
            nx, ny = self._call(0.0, 0.0, step)
            dist = math.hypot(nx, ny)
            self.assertGreater(dist, 0, f'step {step} tidak menghasilkan offset!')

    def test_step6_wraps_to_step1(self):
        """step 6 harus sama dengan step 1 (modulo 5 offsets)."""
        nx6, ny6 = self._call(0.0, 0.0, 6)
        nx1, ny1 = self._call(0.0, 0.0, 1)
        self.assertAlmostEqual(nx6, nx1)
        self.assertAlmostEqual(ny6, ny1)


# ────────────────────────────────────────────────────────────────────────────
# Tests: _load_semantic_map
# ────────────────────────────────────────────────────────────────────────────

class TestLoadSemanticMap(unittest.TestCase):
    def _make_self_with_path(self, path: str):
        ns = types.SimpleNamespace()
        ns.map_path = path
        ns.get_logger = lambda: MagicMock()
        return ns

    def _call(self, path):
        return ObjectGoalNav._load_semantic_map(self._make_self_with_path(path))

    def test_load_valid_yaml(self):
        data = {
            'chair_2':        {'x': 1.23, 'y': 4.56, 'class_id': 0, 'cell_count': 30},
            'refrigerator_1': {'x': 3.0,  'y': -1.0, 'class_id': 3, 'cell_count': 25},
        }
        with tempfile.NamedTemporaryFile(suffix='.yaml', mode='w', delete=False) as f:
            yaml.dump(data, f)
            fname = f.name
        try:
            result = self._call(fname)
            self.assertEqual(len(result), 2)
            self.assertIn('chair_2', result)
            self.assertAlmostEqual(result['chair_2']['x'], 1.23)
            self.assertAlmostEqual(result['chair_2']['y'], 4.56)
        finally:
            os.unlink(fname)

    def test_file_not_found(self):
        result = self._call('/path/yang/tidak/ada.yaml')
        self.assertEqual(result, {})

    def test_empty_yaml(self):
        with tempfile.NamedTemporaryFile(suffix='.yaml', mode='w', delete=False) as f:
            f.write('')
            fname = f.name
        try:
            result = self._call(fname)
            self.assertEqual(result, {})
        finally:
            os.unlink(fname)

    def test_yaml_fields_accessible_like_object_goal_nav(self):
        """Akses fields seperti yang dilakukan object_goal_nav.py."""
        data = {'tv_4': {'x': 2.0, 'y': 3.0, 'class_id': 1, 'cell_count': 50}}
        with tempfile.NamedTemporaryFile(suffix='.yaml', mode='w', delete=False) as f:
            yaml.dump(data, f)
            fname = f.name
        try:
            result = self._call(fname)
            obj = result['tv_4']
            x, y = obj['x'], obj['y']   # seperti navigate_to_object()
            cell_count = obj['cell_count']   # seperti _print_available_objects()
            self.assertAlmostEqual(x, 2.0)
            self.assertAlmostEqual(y, 3.0)
            self.assertEqual(cell_count, 50)
        finally:
            os.unlink(fname)


# ────────────────────────────────────────────────────────────────────────────
# Tests: Retry boundary logic
# ────────────────────────────────────────────────────────────────────────────

class TestRetryLogic(unittest.TestCase):
    def test_max_retry_zero_stops_immediately(self):
        s = _make_self(max_retry=0)
        self.assertFalse(0 < s.max_retry)

    def test_max_retry_five_allows_five_retries(self):
        s = _make_self(max_retry=5)
        for r in range(5):   # retry 0,1,2,3,4 semua valid
            self.assertTrue(r < s.max_retry)
        self.assertFalse(5 < s.max_retry)   # retry ke-5 harus stop

    def test_max_retry_boundary(self):
        s = _make_self(max_retry=3)
        # Simulasi loop retry seperti _result_cb
        navigated = []
        for retry in range(s.max_retry + 1):
            if retry < s.max_retry:
                navigated.append(retry)
        self.assertEqual(navigated, [0, 1, 2])   # 3 retry


def main():
    print('test_object_goal_nav.py — unit tests object_goal_nav.py\n')
    loader = unittest.TestLoader()
    suite = unittest.TestSuite([
        loader.loadTestsFromTestCase(TestIsInMap),
        loader.loadTestsFromTestCase(TestClampToMap),
        loader.loadTestsFromTestCase(TestOffsetTowardCenter),
        loader.loadTestsFromTestCase(TestLoadSemanticMap),
        loader.loadTestsFromTestCase(TestRetryLogic),
    ])
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)


if __name__ == '__main__':
    main()
