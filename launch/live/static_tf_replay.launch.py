"""
static_tf_replay.launch.py — [H2] Static TF untuk mode REPLAY rosbag.

TERPISAH dari static_tf_live.launch.py (tidak diedit sama sekali) —
mode live sengaja MENGANDALKAN RealSense driver untuk publish
camera_link → camera_{color,depth}_optical_frame secara otomatis; itu
tidak mungkin terjadi saat replay bag (tidak ada kamera fisik). File ini
melengkapi transform yang hilang itu, KHUSUS untuk skenario replay.

Isi (5 transform, semuanya static):
  1. a200_1060/base_link → velodyne                  (sama dgn static_tf_live)
  2. a200_1060/base_link → camera_link                (sama dgn static_tf_live)
  3. base_link → a200_1060/base_link                  (bridge alias, sama)
  4. camera_link → camera_color_optical_frame          [BARU, khusus replay]
  5. camera_link → camera_depth_optical_frame          [BARU, khusus replay]

Quaternion (4) & (5): (-0.5, 0.5, -0.5, 0.5) — didokumentasikan di komentar
static_tf_live.launch.py ("dari RealSense internal calibration", "Rosbag
konfirmasi"). Diverifikasi ulang: representasi ini EKUIVALEN dengan RPY
(roll=-pi/2, pitch=0, yaw=-pi/2) yang dipakai di rtabmap_rosbag.launch.py
(repo lama, terbukti jalan) — dicek numerik, bukan diasumsikan sama.

Dipakai --x/--y/--z/--qx/--qy/--qz/--qw/--frame-id/--child-frame-id
(named args, bukan positional) — dikonfirmasi ini gaya yang didukung
resmi di ros2 run tf2_ros static_transform_publisher --help pada image
osrf/ros:humble-desktop-full yang dipakai container ini; gaya positional
lama masih jalan tapi sudah deprecated (warning saat startup).
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def _static_tf(name, x, y, z, frame_id, child_frame_id,
                qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[
            '--x', str(x), '--y', str(y), '--z', str(z),
            '--qx', str(qx), '--qy', str(qy), '--qz', str(qz), '--qw', str(qw),
            '--frame-id', frame_id, '--child-frame-id', child_frame_id,
        ],
        parameters=[{'use_sim_time': True}],   # replay pakai /clock dari ros2 bag play --clock
        output='screen',
    )


def generate_launch_description():

    # ── 1. Velodyne VLP-32C — x=0.1, z=0.5, identity rotation ────────────
    tf_base_velodyne = _static_tf(
        'tf_base_to_velodyne', 0.1, 0, 0.5,
        'a200_1060/base_link', 'velodyne')

    # ── 2. RealSense D455 (link, bukan optical) — x=0.2, z=0.25 ──────────
    tf_base_camera = _static_tf(
        'tf_base_camera', 0.2, 0, 0.25,
        'a200_1060/base_link', 'camera_link')

    # ── 3. Bridge namespace alias ─────────────────────────────────────────
    tf_bridge_base = _static_tf(
        'tf_bridge_base', 0, 0, 0,
        'base_link', 'a200_1060/base_link')

    # ── 4/5. Optical frame — TIDAK tersedia dari RealSense driver saat
    #        replay (tidak ada kamera fisik), jadi dipublish manual di sini.
    tf_camera_color_optical = _static_tf(
        'tf_camera_color_optical', 0, 0, 0,
        'camera_link', 'camera_color_optical_frame',
        qx=-0.5, qy=0.5, qz=-0.5, qw=0.5)

    tf_camera_depth_optical = _static_tf(
        'tf_camera_depth_optical', 0, 0, 0,
        'camera_link', 'camera_depth_optical_frame',
        qx=-0.5, qy=0.5, qz=-0.5, qw=0.5)

    return LaunchDescription([
        tf_base_velodyne,
        tf_base_camera,
        tf_bridge_base,
        tf_camera_color_optical,
        tf_camera_depth_optical,
    ])
