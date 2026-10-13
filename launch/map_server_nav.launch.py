#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════╗
║  map_server_nav.launch.py — Map Server + AMCL (nav ringan)     ║
║                                                                  ║
║  Pengganti RTABMap di ./run.sh nav. Load map.pgm/yaml →        ║
║  publish /map, lalu AMCL localize robot dan publish             ║
║  TF map → odom.                                                  ║
║                                                                  ║
║  Dimanage oleh lifecycle_manager_localization (autostart).      ║
║                                                                  ║
║  Usage:                                                          ║
║    ros2 launch map_server_nav.launch.py \                       ║
║        map_yaml:=/path/to/lab_map.yaml                          ║
╚══════════════════════════════════════════════════════════════════╝
"""

import os
import math
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_amcl_node(context, map_yaml, initial_x, initial_y, initial_yaw, set_initial_pose):
    ix    = float(context.perform_substitution(initial_x))
    iy    = float(context.perform_substitution(initial_y))
    iyaw  = float(context.perform_substitution(initial_yaw))
    do_set = context.perform_substitution(set_initial_pose).lower() == 'true'

    return [Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time':               False,
            'alpha1':                     0.2,
            'alpha2':                     0.2,
            'alpha3':                     0.2,
            'alpha4':                     0.2,
            'alpha5':                     0.2,
            'base_frame_id':              'a200_1060/base_link',
            'beam_skip_distance':         0.5,
            'beam_skip_error_threshold':  0.9,
            'beam_skip_threshold':        0.3,
            'do_beamskip':                False,
            'global_frame_id':            'map',
            'lambda_short':               0.1,
            'laser_likelihood_max_dist':  2.0,
            'laser_max_range':            100.0,
            'laser_min_range':            -1.0,
            'laser_model_type':           'likelihood_field',
            'max_beams':                  60,
            'max_particles':              2000,
            'min_particles':              500,
            'odom_frame_id':              'odom',
            'pf_err':                     0.05,
            'pf_z':                       0.99,
            'recovery_alpha_fast':        0.0,
            'recovery_alpha_slow':        0.0,
            'resample_interval':          1,
            'robot_model_type':           'nav2_amcl::DifferentialMotionModel',
            'save_pose_rate':             0.02,
            'sigma_hit':                  0.2,
            'tf_broadcast':               True,
            'transform_tolerance':        2.0,
            'update_min_a':               0.05,
            'update_min_d':               0.05,
            'z_hit':                      0.5,
            'z_max':                      0.05,
            'z_rand':                     0.5,
            'z_short':                    0.05,
            'scan_topic':                 '/scan',
            'map_topic':                  '/map',
            'set_initial_pose':           do_set,
            'initial_pose':               {'x': ix, 'y': iy, 'z': 0.0, 'yaw': iyaw},
        }],
        remappings=[
            ('scan', '/scan'),
            ('map',  '/map'),
        ],
    )]


def generate_launch_description():

    # ── Argumen ────────────────────────────────────────────────────
    declare_map_yaml = DeclareLaunchArgument(
        'map_yaml',
        default_value=os.path.join(os.path.expanduser('~'), '.ros', 'lab_map.yaml'),
        description='Path ke map YAML (nav2_map_server format)'
    )
    declare_initial_x = DeclareLaunchArgument(
        'initial_x', default_value='0.0',
        description='Pose awal AMCL — x (meter)')
    declare_initial_y = DeclareLaunchArgument(
        'initial_y', default_value='0.0',
        description='Pose awal AMCL — y (meter)')
    declare_initial_yaw = DeclareLaunchArgument(
        'initial_yaw', default_value='0.0',
        description='Pose awal AMCL — yaw (radian)')
    declare_set_initial_pose = DeclareLaunchArgument(
        'set_initial_pose', default_value='true',
        description='true=AMCL langsung init ke initial_x/y/yaw | false=tunggu /initialpose')

    map_yaml          = LaunchConfiguration('map_yaml')
    initial_x         = LaunchConfiguration('initial_x')
    initial_y         = LaunchConfiguration('initial_y')
    initial_yaw       = LaunchConfiguration('initial_yaw')
    set_initial_pose  = LaunchConfiguration('set_initial_pose')

    # ── Map Server ─────────────────────────────────────────────────
    # Load map.pgm + map.yaml → publish /map (OccupancyGrid)
    # Menggantikan RTABMap sebagai sumber /map untuk AMCL dan Nav2.
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml,
            'topic_name':    'map',
            'frame_id':      'map',
        }],
    )

    amcl_opaque = OpaqueFunction(
        function=_make_amcl_node,
        args=[map_yaml, initial_x, initial_y, initial_yaw, set_initial_pose],
    )

    # ── Lifecycle Manager ──────────────────────────────────────────
    # Manage map_server + amcl bersama. Dipisah dari
    # lifecycle_manager_navigation (Nav2) agar tidak konflik.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart':    True,
            'bond_timeout': 30.0,
            'node_names':   ['map_server', 'amcl'],
        }],
    )

    return LaunchDescription([
        declare_map_yaml,
        declare_initial_x,
        declare_initial_y,
        declare_initial_yaw,
        declare_set_initial_pose,
        map_server_node,
        TimerAction(
            period=2.0,
            actions=[
                LogInfo(msg='>>> [T+2s] Launching AMCL...'),
                amcl_opaque,
            ],
        ),
        lifecycle_manager,
    ])
