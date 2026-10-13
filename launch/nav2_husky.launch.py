#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════╗
║  nav2_husky.launch.py — Nav2 untuk Husky A200 + RTABMap        ║
║                                                                  ║
║  Catatan:                                                        ║
║  - map_server TIDAK dilaunched — RTABMap publish /map           ║
║  - odom: /a200_1060/platform/odom/filtered                      ║
║  - cmd_vel: /cmd_vel (Husky driver subscribe di sini)           ║
║  - Velodyne VLP-32C: /velodyne_points (PointCloud2)             ║
║  - use_amcl:=true  → include AMCL + manage via lifecycle       ║
║  - use_amcl:=false → skip AMCL                                  ║
║                                                                  ║
║  Usage:                                                          ║
║    ros2 launch nav2_husky.launch.py [use_amcl:=true]           ║
╚══════════════════════════════════════════════════════════════════╝
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

# Path ke params file — satu level di atas launch/ → rtabmap_allinone_live/config/
PARAMS_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'config', 'nav2_params.yaml'
)

# Path ke amcl_husky.launch.py — di direktori yang sama
AMCL_LAUNCH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    'amcl_husky.launch.py'
)

# [2026-07-09] Mask keepout dari scene_zone grid (area cream/open_area =
# uncovered saat scanning + wall/unknown ditandai occupied). Regenerate via
# semantic/scene_zone_to_keepout_mask.py kalau scene_zone grid berubah.
KEEPOUT_MASK_YAML = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'output', 'scene_zone_test', 'keepout_mask.yaml'
)

# [2026-07-08] Custom BT (SmoothPath disisipkan) DIMATIKAN — dites di robot
# fisik, menyebabkan "Extrapolation Error: Lookup would require extrapolation
# into the past" berulang (TF request pakai timestamp beku dari saat goal
# diterima) -> cmd_vel TIDAK PERNAH terpublish, robot 100% tidak gerak.
# A/B test: BT bawaan nav2 (tanpa smoothing) -> number_of_recoveries 0,
# jalan normal. Balik ke BT bawaan nav2 (default_nav_to_pose_bt_xml TIDAK
# di-override, biarkan bt_navigator pakai default internalnya) sampai bug
# timestamp di custom BT ditemukan & difix. File custom BT tetap disimpan
# di config/behavior_trees/navigate_to_pose_w_smoothing.xml kalau mau
# dicoba lagi nanti setelah bug-nya jelas.


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    declare_use_amcl = DeclareLaunchArgument(
        'use_amcl', default_value='true',
        description='true=include AMCL + manage via lifecycle | false=skip AMCL')

    # [2026-07-14] Hook opsional, default KOSONG = tidak ada efek sama sekali
    # di produksi. Dipakai nav2_sim.launch.py di ROS2 Jazzy (laptop dev) —
    # paket apt nav2_smac_planner/nav2_navfn_planner/nav2_bt_navigator di
    # Jazzy menghapus atribut name= dari plugin XML-nya, jadi lookup name
    # "package/ClassName" di nav2_params.yaml gagal resolve di situ (lihat
    # config/nav2_params_jazzy_plugin_overrides.yaml). nav2_params.yaml
    # SENGAJA TIDAK DIUBAH (sudah divalidasi di robot fisik/Humble).
    declare_extra_params = DeclareLaunchArgument(
        'extra_params_file', default_value='',
        description='Path params file tambahan (override sebagian key di atas '
                     'PARAMS_FILE) — dibiarkan kosong di produksi.')

    def build_nav2(context, *args, **kwargs):
        use_amcl = context.launch_configurations.get('use_amcl', 'true')
        extra_params_file = context.launch_configurations.get('extra_params_file', '')
        planner_params = [PARAMS_FILE, extra_params_file] if extra_params_file else [PARAMS_FILE]
        bt_nav_params = [PARAMS_FILE, extra_params_file] if extra_params_file else [PARAMS_FILE]

        node_names = [
            'keepout_mask_server',
            'costmap_filter_info_server',
            'controller_server',
            'smoother_server',
            'planner_server',
            'behavior_server',
            'bt_navigator',
            'waypoint_follower',
            'velocity_smoother',
        ]

        actions = [
            # ── Keepout mask (scene_zone grid: cream/wall/unknown = occupied) ──
            # Map_server terpisah dari /map RTABMap — cuma publish mask
            # statis ini di /keepout_mask, dikonsumsi KeepoutFilter di
            # kedua costmap (lihat config/nav2_params.yaml).
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='keepout_mask_server',
                output='screen',
                parameters=[{
                    'yaml_filename': KEEPOUT_MASK_YAML,
                    'topic_name': 'keepout_mask',
                    'frame_id': 'map',
                }],
            ),
            Node(
                package='nav2_map_server',
                executable='costmap_filter_info_server',
                name='costmap_filter_info_server',
                output='screen',
                parameters=[{
                    'filter_info_topic': '/keepout_filter_info',
                    'mask_topic': '/keepout_mask',
                    'type': 0,
                    'base': 0.0,
                    'multiplier': 1.0,
                }],
            ),

            # ── Controller Server ──────────────────────────────────────
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[PARAMS_FILE],
                remappings=[
                    ('cmd_vel', '/cmd_vel_nav'),
                ],
            ),

            # ── Smoother Server ────────────────────────────────────────
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=[PARAMS_FILE],
            ),

            # ── Planner Server ─────────────────────────────────────────
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=planner_params,
            ),

            # ── Behavior Server ────────────────────────────────────────
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[PARAMS_FILE],
                remappings=[
                    ('cmd_vel', '/cmd_vel_nav'),
                ],
            ),

            # ── BT Navigator ───────────────────────────────────────────
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=bt_nav_params,
                remappings=[
                    ('odom', '/a200_1060/platform/odom/filtered'),
                ],
            ),

            # ── Waypoint Follower ──────────────────────────────────────
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[PARAMS_FILE],
            ),

            # ── Velocity Smoother ──────────────────────────────────────
            # Terima dari controller/behavior (/cmd_vel_nav),
            # output ke twist_mux Husky (/a200_1060/cmd_vel).
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[PARAMS_FILE],
                remappings=[
                    ('cmd_vel',          '/cmd_vel_nav'),
                    ('cmd_vel_smoothed', '/a200_1060/cmd_vel'),
                ],
            ),
        ]

        if use_amcl == 'true':
            node_names.insert(0, 'amcl')
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(AMCL_LAUNCH)
                )
            )

        # ── Lifecycle Manager ──────────────────────────────────────
        actions.append(
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart':    True,
                    'bond_timeout': 30.0,
                    'node_names':   node_names,
                }],
            )
        )

        return actions

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        declare_use_amcl,
        declare_extra_params,
        SetParameter(name='use_sim_time', value=use_sim_time),
        OpaqueFunction(function=build_nav2),
    ])
