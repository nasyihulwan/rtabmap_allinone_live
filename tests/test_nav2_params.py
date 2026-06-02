#!/usr/bin/env python3
"""
test_nav2_params.py — Validasi nav2_params.yaml tanpa ROS

Cek:
  1. File valid YAML
  2. robot_radius cukup untuk Husky A200 (>= 0.33m)
  3. Velodyne VLP-32C: topic /velodyne_points, data_type PointCloud2
  4. use_sim_time: false di semua komponen
  5. lifecycle_manager node list lengkap
  6. inflation_layer terkonfigurasi
"""

import sys
import os
import yaml
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NAV2_YAML = os.path.join(ROOT, 'config', 'nav2_params.yaml')

HUSKY_MIN_RADIUS = 0.33     # Husky A200 half-width = 0.335m
REQUIRED_NAV_NODES = {
    'controller_server', 'smoother_server', 'planner_server',
    'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother',
}


class TestNav2Params(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        with open(NAV2_YAML, 'r') as f:
            cls.cfg = yaml.safe_load(f)

    def test_yaml_loads(self):
        self.assertIsInstance(self.cfg, dict, "nav2_params.yaml harus berupa dict")

    # ── robot_radius ──────────────────────────────────────────────────────
    def test_local_costmap_robot_radius(self):
        r = (self.cfg['local_costmap']['local_costmap']
             ['ros__parameters']['robot_radius'])
        self.assertGreaterEqual(
            r, HUSKY_MIN_RADIUS,
            f'local_costmap robot_radius={r} terlalu kecil '
            f'(Husky A200 min={HUSKY_MIN_RADIUS})')

    def test_global_costmap_robot_radius(self):
        r = (self.cfg['global_costmap']['global_costmap']
             ['ros__parameters']['robot_radius'])
        self.assertGreaterEqual(
            r, HUSKY_MIN_RADIUS,
            f'global_costmap robot_radius={r} terlalu kecil '
            f'(Husky A200 min={HUSKY_MIN_RADIUS})')

    # ── Velodyne topic ────────────────────────────────────────────────────
    def test_voxel_layer_topic(self):
        params = (self.cfg['local_costmap']['local_costmap']
                  ['ros__parameters'])
        obs_sources = params.get('voxel_layer', {}).get('observation_sources', '')
        src_name = obs_sources if isinstance(obs_sources, str) else list(obs_sources)[0]
        topic = params['voxel_layer'][src_name]['topic']
        self.assertEqual(
            topic, '/velodyne_points',
            f'voxel_layer topic harus /velodyne_points bukan {topic!r}. '
            'VLP-32C tidak publish LaserScan!')

    def test_voxel_layer_data_type(self):
        params = (self.cfg['local_costmap']['local_costmap']
                  ['ros__parameters'])
        obs_sources = params.get('voxel_layer', {}).get('observation_sources', '')
        src_name = obs_sources if isinstance(obs_sources, str) else list(obs_sources)[0]
        dtype = params['voxel_layer'][src_name]['data_type']
        self.assertEqual(
            dtype, 'PointCloud2',
            f'voxel_layer data_type harus PointCloud2 bukan {dtype!r}. '
            'VLP-32C publish PointCloud2!')

    def test_voxel_layer_min_obstacle_height(self):
        params = (self.cfg['local_costmap']['local_costmap']
                  ['ros__parameters'])
        obs_sources = params.get('voxel_layer', {}).get('observation_sources', '')
        src_name = obs_sources if isinstance(obs_sources, str) else list(obs_sources)[0]
        src = params['voxel_layer'][src_name]
        self.assertIn(
            'min_obstacle_height', src,
            'min_obstacle_height harus di-set untuk filter ground dari VLP-32C')
        self.assertGreater(src['min_obstacle_height'], 0.0,
                           'min_obstacle_height harus > 0 untuk filter ground')

    # ── use_sim_time ──────────────────────────────────────────────────────
    def _check_sim_time(self, component: str):
        """Helper: pastikan use_sim_time: false di komponen yang diberikan."""
        node_cfg = self.cfg.get(component, {})
        # handle nested (local_costmap/global_costmap)
        if component in ('local_costmap', 'global_costmap'):
            node_cfg = node_cfg.get(component, {})
        params = node_cfg.get('ros__parameters', {})
        val = params.get('use_sim_time', None)
        self.assertFalse(
            val,
            f'{component}.use_sim_time harus false untuk live mode (got {val})')

    def test_use_sim_time_bt_navigator(self):
        self._check_sim_time('bt_navigator')

    def test_use_sim_time_controller_server(self):
        self._check_sim_time('controller_server')

    def test_use_sim_time_planner_server(self):
        self._check_sim_time('planner_server')

    def test_use_sim_time_local_costmap(self):
        self._check_sim_time('local_costmap')

    def test_use_sim_time_global_costmap(self):
        self._check_sim_time('global_costmap')

    def test_use_sim_time_behavior_server(self):
        self._check_sim_time('behavior_server')

    # ── Lifecycle manager ─────────────────────────────────────────────────
    def test_lifecycle_manager_nodes(self):
        nodes = set(self.cfg['lifecycle_manager_navigation']
                    ['ros__parameters']['node_names'])
        missing = REQUIRED_NAV_NODES - nodes
        self.assertFalse(
            missing,
            f'lifecycle_manager_navigation kekurangan nodes: {missing}')

    def test_lifecycle_manager_autostart(self):
        autostart = (self.cfg['lifecycle_manager_navigation']
                     ['ros__parameters']['autostart'])
        self.assertTrue(autostart, 'lifecycle_manager autostart harus true')

    # ── Inflation layer ───────────────────────────────────────────────────
    def test_local_inflation_configured(self):
        params = (self.cfg['local_costmap']['local_costmap']
                  ['ros__parameters'])
        self.assertIn('inflation_layer', params,
                      'local_costmap harus punya inflation_layer')
        self.assertIn('inflation_radius', params['inflation_layer'],
                      'inflation_layer harus punya inflation_radius')

    def test_global_inflation_configured(self):
        params = (self.cfg['global_costmap']['global_costmap']
                  ['ros__parameters'])
        self.assertIn('inflation_layer', params,
                      'global_costmap harus punya inflation_layer')
        self.assertIn('inflation_radius', params['inflation_layer'],
                      'inflation_layer harus punya inflation_radius')

    # ── Controller DWB ────────────────────────────────────────────────────
    def test_dwb_max_vel_x(self):
        follow = (self.cfg['controller_server']['ros__parameters']
                  ['FollowPath'])
        max_v = follow.get('max_vel_x', 0)
        self.assertGreater(max_v, 0.0, 'max_vel_x harus > 0')
        self.assertLessEqual(max_v, 1.0, 'max_vel_x terlalu besar untuk Husky indoor')

    # ── Planner ───────────────────────────────────────────────────────────
    def test_planner_allow_unknown(self):
        planner = (self.cfg['planner_server']['ros__parameters']
                   ['GridBased'])
        self.assertTrue(planner.get('allow_unknown', False),
                        'allow_unknown harus true agar RTABMap unknown space bisa dilewati')


def main():
    print('test_nav2_params.py — validasi config/nav2_params.yaml\n')
    loader = unittest.TestLoader()
    suite = loader.loadTestsFromTestCase(TestNav2Params)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)


if __name__ == '__main__':
    main()
