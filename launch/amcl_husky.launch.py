#!/usr/bin/env python3
"""
amcl_husky.launch.py — AMCL untuk Husky A200 + RTABMap

Subscribe /scan (Velodyne VLP-32C) dan /map (RTABMap).
Publish TF map → odom (tf_broadcast: True).
Node ini di-include dari nav2_husky.launch.py dan DIMANAGE oleh
lifecycle_manager_navigation (node_names menyertakan 'amcl').
Delay 3s untuk memberi waktu RTABMap mempublish map sebelum AMCL aktif.
"""

from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo
from launch_ros.actions import Node


def generate_launch_description():

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'alpha1': 0.2,
            'alpha2': 0.2,
            'alpha3': 0.2,
            'alpha4': 0.2,
            'alpha5': 0.2,
            'base_frame_id': 'a200_1060/base_link',
            'beam_skip_distance': 0.5,
            'beam_skip_error_threshold': 0.9,
            'beam_skip_threshold': 0.3,
            'do_beamskip': False,
            'global_frame_id': 'map',
            'lambda_short': 0.1,
            'laser_likelihood_max_dist': 2.0,
            'laser_max_range': 100.0,
            'laser_min_range': -1.0,
            'laser_model_type': 'likelihood_field',
            'max_beams': 60,
            'max_particles': 2000,
            'min_particles': 500,
            'odom_frame_id': 'odom',
            'pf_err': 0.05,
            'pf_z': 0.99,
            'recovery_alpha_fast': 0.0,
            'recovery_alpha_slow': 0.0,
            'resample_interval': 1,
            'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
            'save_pose_rate': 0.5,
            'sigma_hit': 0.2,
            'tf_broadcast': True,
            'transform_tolerance': 1.0,
            'update_min_a': 0.2,
            'update_min_d': 0.25,
            'z_hit': 0.5,
            'z_max': 0.05,
            'z_rand': 0.5,
            'z_short': 0.05,
            'scan_topic': '/scan',
            'map_topic': '/map',
            'set_initial_pose': True,
            'initial_pose': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0},
        }],
        remappings=[
            ('scan', '/scan'),
            ('map',  '/map'),
        ],
    )

    return LaunchDescription([
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg='>>> [T+3s] AMCL (map->odom TF via particle filter)...'),
                amcl_node,
            ],
        ),
    ])
