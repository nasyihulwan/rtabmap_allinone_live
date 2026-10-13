#!/usr/bin/env python3

# ═══════════════════════════════════════════════════════════════════════
# semantic_locnav.launch.py
#
# Semantic pipeline RINGAN untuk fase C (locnav).
# Hanya node yang dibutuhkan saat navigasi ke objek yang sudah dipetakan.
# YOLO, SegFormer, projector, grid, exporter TIDAK dijalankan.
#
# Requires:
#   ~/.ros/semantic_map.yaml  — hasil fase A (mapping)
#   /velodyne_points          — untuk obstacle stop
#
# Nodes:
#   semantic_map_viz    — publish MarkerArray ke RViz dari YAML
#   object_goal_nav     — terima /object_goal → kirim Nav2 goal
#   obstacle_stop_node  — safety stop dari lidar
#   person_detector     — [2026-07-13] deteksi + tracking orang (model COCO
#                          kedua, model custom gak punya kelas 'person') utk
#                          dynamic obstacle avoidance behavioral layer.
#                          Model custom (yolo_detector.py)/SegFormer TIDAK
#                          jalan di launch ringan ini, jadi ada headroom GPU.
#   lab_robot_detector  — [2026-07-13] deteksi + tracking robot lab MOBILE
#                          (dell-e/i-seater/o-seater, model custom .pt task
#                          detect) — robotic-arm DIKECUALIKAN (fixture diam).
# ═══════════════════════════════════════════════════════════════════════

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    script_dir   = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    semantic_dir = os.path.join(script_dir, 'semantic')
    person_model_default = os.path.join(script_dir, 'models', 'yolo11n.pt')
    lab_robot_model_default = os.path.join(
        script_dir, 'models', 'best_brin_yolo11m_v1_20260517_0724.pt')

    person_device_arg = DeclareLaunchArgument('person_device', default_value='0')
    person_device = LaunchConfiguration('person_device')
    lab_robot_device_arg = DeclareLaunchArgument('lab_robot_device', default_value='0')
    lab_robot_device = LaunchConfiguration('lab_robot_device')

    # ── Node 1: Semantic Map Viz ──────────────────────────────────────────
    semantic_map_viz = ExecuteProcess(
        cmd=[
            'python3', os.path.join(semantic_dir, 'semantic_map_viz.py'),
            '--ros-args',
        ],
        output='screen',
    )

    # ── Node 2: Object Goal Nav (delay 2s — viz harus up dulu) ───────────
    object_goal_nav = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'python3', os.path.join(script_dir, 'object_goal_nav.py'),
                    '--ros-args',
                ],
                output='screen',
            )
        ]
    )

    # ── Node 3: Obstacle Stop (langsung dari awal) ────────────────────────
    obstacle_stop = ExecuteProcess(
        cmd=[
            'python3', os.path.join(semantic_dir, 'obstacle_stop_node.py'),
            '--ros-args',
        ],
        output='screen',
    )

    # ── Node 4: Person Detector (delay 2s — camera driver harus up) ───────
    person_detector = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'python3', os.path.join(semantic_dir, 'person_detector.py'),
                    '--ros-args',
                    '-p', ['model_path:=', person_model_default],
                    '-p', ["device:='", person_device, "'"],
                    '-p', 'process_every_n:=5',
                ],
                output='screen',
            )
        ]
    )

    # ── Node 5: Lab Robot Detector (delay 2s — camera driver harus up) ────
    lab_robot_detector = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'python3', os.path.join(semantic_dir, 'lab_robot_detector.py'),
                    '--ros-args',
                    '-p', ['model_path:=', lab_robot_model_default],
                    '-p', ["device:='", lab_robot_device, "'"],
                    '-p', 'process_every_n:=5',
                ],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        person_device_arg,
        lab_robot_device_arg,
        semantic_map_viz,
        object_goal_nav,
        obstacle_stop,
        person_detector,
        lab_robot_detector,
    ])
