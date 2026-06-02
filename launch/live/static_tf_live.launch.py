"""
static_tf_live.launch.py
Static TF publisher untuk live mode di AGX Orin.

Untuk rosbag: TF velodyne sudah terekam di /tf_static dari PC Husky.
Untuk live  : semua TF harus dipublish manual di sini.

Frame yang dipublish (3 saja):
  - a200_1060/base_link → velodyne     (x=0.1, z=0.5, dikonfirmasi dari rosbag)
  - a200_1060/base_link → camera_link  (x=0.2, z=0.25, dari rtabmap launch)
  - base_link → a200_1060/base_link    (bridge alias)

TIDAK dipublish (RealSense driver handle otomatis di live mode):
  - camera_link → camera_depth_frame → camera_depth_optical_frame
  - camera_link → camera_color_frame → camera_color_optical_frame
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # ── Velodyne VLP-32C ──────────────────────────────────────────
    # x=0.1 (10cm depan), y=0, z=0.5 (50cm atas) dari base_link
    # Nilai dari PC Husky launch file — sesuaikan kalau posisi fisik berubah
    tf_base_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_velodyne',
        arguments=[
            '0.1', '0', '0.5',   # x y z (meter)
            '0', '0', '0',        # roll pitch yaw (radian)
            'a200_1060/base_link', 'velodyne'
        ],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    # ── RealSense D455 ────────────────────────────────────────────
    # x=0.2 (20cm depan), y=0, z=0.25 (25cm atas) dari base_link
    # Sesuaikan dengan pengukuran fisik posisi kamera di Husky
    tf_base_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_camera',
        arguments=[
            '0.2', '0', '0.25',  # x y z (meter)
            '0', '0', '0',        # roll pitch yaw (radian)
            'a200_1060/base_link', 'camera_link'
        ],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    # ── base_link alias → a200_1060/base_link ────────────────────
    # CATATAN: camera_color_optical_frame dan camera_depth_optical_frame
    # TIDAK dipublish di sini — RealSense driver publish otomatis di live mode.
    # Rosbag konfirmasi: camera_link → camera_depth_frame → camera_depth_optical_frame
    #                    camera_link → camera_color_frame → camera_color_optical_frame
    # (rotation: -0.5, 0.5, -0.5, 0.5 dari RealSense internal calibration)
    # Bridge agar RTABMap bisa lookup TF dengan namespace
    tf_bridge_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_bridge_base',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'base_link', 'a200_1060/base_link'
        ],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    return LaunchDescription([
        tf_base_velodyne,   # a200_1060/base_link → velodyne (x=0.1, z=0.5) ✓ rosbag confirmed
        tf_base_camera,     # a200_1060/base_link → camera_link (x=0.2, z=0.25)
        tf_bridge_base,     # base_link → a200_1060/base_link (alias)
        # camera optical frames → dipublish otomatis oleh RealSense driver di live mode
    ])
