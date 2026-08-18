import os
"""
rtabmap_replay.launch.py — [H2] RTAB-Map untuk bag REPLAY (bukan live sensor).

Adaptasi dari rtabmap_live.launch.py — SAMA PERSIS parameter RTAB-Map-nya
(Kp/Vis/RGBD/Grid/Reg/Icp) supaya hasil replay sebanding dengan pipeline
live yang sesungguhnya dipakai sekarang. rtabmap_live.launch.py sendiri
TIDAK diedit.

Perbedaan dari rtabmap_live.launch.py:
  1. use_sim_time = True di semua node (pakai /clock dari ros2 bag play --clock)
  2. Tambah rosbag_play (ExecuteProcess) di awal
  3. Topic depth diremap ke output decompress_depth_node.py
     (/camera/camera/aligned_depth_to_color/image_raw) alih-alih topic live
     RealSense (/camera/camera/depth/image_rect_raw) — beda source, sama nama
     akhir seperti live sebetulnya (aligned_depth_to_color juga dipakai live,
     lihat semantic_projector.py), jadi ini justru match, bukan divergen.
  4. rgb/image tetap /camera/camera/color/image_raw — sama seperti live,
     hanya sumbernya jadi image_transport republish (bag compressed → raw),
     bukan RealSense driver langsung.
  5. TIDAK menjalankan static TF kamera/velodyne di sini — itu tanggung
     jawab static_tf_replay.launch.py (dijalankan terpisah, lihat file itu).
  6. rviz & rtabmap_viz default 'false' (container headless, CPU-only, tidak
     ada display).

PRASYARAT (sebelum jalankan launch ini):
  - static_tf_replay.launch.py sudah/akan jalan bareng (base_link<->camera/
    velodyne + camera optical frames)
  - image_transport republish compressed->raw untuk RGB sudah/akan jalan
  - decompress_depth_node.py sudah/akan jalan
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo, OpaqueFunction
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOT_BASE_FRAME = 'a200_1060/base_link'
ODOM_FRAME       = 'odom'
MAP_FRAME        = 'map'


def generate_launch_description():

    bag_path     = LaunchConfiguration('bag_path')
    rate         = LaunchConfiguration('rate')
    rviz         = LaunchConfiguration('rviz')
    db_path      = LaunchConfiguration('db_path')
    use_ekf      = LaunchConfiguration('use_ekf')
    odom_topic   = LaunchConfiguration('odom_topic')
    localization = LaunchConfiguration('localization')

    # ── Argumen ────────────────────────────────────────────────────
    declare_bag_path = DeclareLaunchArgument(
        'bag_path',
        default_value='/root/ws/project/rosbag_launch/bags/rosbag2_2026_04_15-07_59_18',
        description='Path bag di dalam container')
    declare_rate = DeclareLaunchArgument('rate', default_value='1.0')
    declare_rviz = DeclareLaunchArgument('rviz', default_value='false')
    declare_rtabmap_viz = DeclareLaunchArgument(
        'use_rtabmap_viz', default_value='false')
    declare_db_path = DeclareLaunchArgument(
        'db_path', default_value=os.path.expanduser('~/.ros/rtabmap_replay.db'))
    declare_use_ekf = DeclareLaunchArgument(
        'use_ekf', default_value='false',
        description='false=pakai odom/filtered dari bag | true=EKF')
    declare_odom_topic = DeclareLaunchArgument(
        'odom_topic', default_value='/a200_1060/platform/odom/filtered')
    declare_localization = DeclareLaunchArgument(
        'localization', default_value='false',
        description='true=localization mode | false=mapping mode')

    # ══════════════════════════════════════════════════════════════
    # T+0s — ROSBAG PLAY
    # ══════════════════════════════════════════════════════════════
    rosbag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag_path,
            '--clock', '--rate', rate,
            '--read-ahead-queue-size', '1000',
        ],
        output='screen', name='rosbag_play')

    # ══════════════════════════════════════════════════════════════
    # T+2s — RELAY TF + ODOM (bridge namespace a200_1060, sama seperti live)
    # ══════════════════════════════════════════════════════════════
    relay_tf = Node(
        package='topic_tools', executable='relay', name='relay_tf',
        arguments=['/a200_1060/tf', '/tf'],
        parameters=[{'use_sim_time': True}], output='log')

    relay_tf_static = Node(
        package='topic_tools', executable='relay', name='relay_tf_static',
        arguments=['/a200_1060/tf_static', '/tf_static'],
        parameters=[{'use_sim_time': True}], output='screen')

    relay_odom = Node(
        package='topic_tools', executable='relay', name='relay_odom',
        arguments=['/a200_1060/platform/odom', '/odom'],
        parameters=[{'use_sim_time': True}], output='screen')

    relay_imu = Node(
        package='topic_tools', executable='relay', name='relay_imu',
        arguments=['/a200_1060/sensors/imu_0/data', '/imu/data'],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_ekf),
        output='screen')

    # ══════════════════════════════════════════════════════════════
    # T+3s — EKF NODE (opsional, default false — bag punya odom/filtered)
    # ══════════════════════════════════════════════════════════════
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=IfCondition(use_ekf),
        parameters=[{
            'use_sim_time':     True,
            'frequency':        50.0,
            'sensor_timeout':   0.1,
            'two_d_mode':       True,
            'publish_tf':       False,
            'map_frame':        MAP_FRAME,
            'odom_frame':       ODOM_FRAME,
            'base_link_frame':  ROBOT_BASE_FRAME,
            'world_frame':      ODOM_FRAME,
            'odom0': '/odom',
            'odom0_config': [
                True,  True,  False,
                False, False, True,
                True,  True,  False,
                False, False, True,
                False, False, False,
            ],
            'odom0_differential': False,
            'odom0_relative':     False,
            'odom0_queue_size':   10,
            'imu0': '/imu/data',
            'imu0_config': [
                False, False, False,
                False, False, True,
                False, False, False,
                True,  True,  True,
                True,  True,  False,
            ],
            'imu0_differential':  False,
            'imu0_relative':      False,
            'imu0_queue_size':    50,
            'imu0_remove_gravitational_acceleration': True,
        }])

    # ══════════════════════════════════════════════════════════════
    # T+6s — RTAB-MAP (via OpaqueFunction) — parameter SAMA dgn
    # rtabmap_live.launch.py mode non-locnav (mapping penuh)
    # ══════════════════════════════════════════════════════════════
    def launch_rtabmap(context, *args, **kwargs):
        loc         = context.launch_configurations.get('localization', 'false')
        db          = context.launch_configurations.get(
            'db_path', os.path.expanduser('~/.ros/rtabmap_replay.db'))
        odom        = context.launch_configurations.get(
            'odom_topic', '/a200_1060/platform/odom/filtered')
        rviz_flag   = context.launch_configurations.get('rviz', 'false')
        use_viz     = context.launch_configurations.get('use_rtabmap_viz', 'false')
        incremental = 'false' if loc == 'true' else 'true'

        rtabmap_node = Node(
            package='rtabmap_slam', executable='rtabmap', name='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time':         True,   # ← REPLAY MODE
                'database_path':        db,
                'frame_id':             ROBOT_BASE_FRAME,
                'map_frame_id':         MAP_FRAME,
                'odom_frame_id':        ODOM_FRAME,
                'subscribe_depth':      True,
                'subscribe_rgb':        True,
                'subscribe_scan_cloud': True,
                'subscribe_odom_info':  False,
                'approx_sync':              True,
                'approx_sync_max_interval': 6.0,
                'sync_queue_size':          50,
                'topic_queue_size':         50,
                'publish_tf':               True,

                'Mem/STMSize':             '30',
                'Mem/RehearsalSimilarity': '0.45',
                'Mem/NotLinkedNodesKept':  'false',
                'Mem/IncrementalMemory':   incremental,
                'initial_pose':            '0 0 0 0 0 0',
                'Mem/InitWMWithAllNodes':  'false',

                'Kp/DetectorStrategy': '6',
                'Vis/FeatureType':     '6',
                'Kp/MaxFeatures':      '600',
                'Vis/MinInliers':      '20',
                'Vis/InlierDistance':  '0.1',
                'Vis/EstimationType':  '1',
                'Vis/MaxDepth':        '10.0',

                'RGBD/OptimizeFromGraphEnd': 'false',
                'RGBD/OptimizeMaxError':     '3.0',
                'Optimizer/Strategy':        '1',
                'Optimizer/Iterations':      '20',
                'Optimizer/Robust':          'true',

                'RGBD/ProximityBySpace':          'true',
                'RGBD/ProximityMaxGraphDepth':    '0',
                'RGBD/ProximityPathMaxNeighbors': '5',
                'RGBD/NeighborLinkRefining':      'true',
                'RGBD/AngularUpdate':             '0.01',
                'RGBD/LinearUpdate':              '0.01',

                'RGBD/CreateOccupancyGrid': 'true',
                'Grid/Sensor':              '2',
                'Grid/3D':                  'false',
                'Grid/MaxObstacleHeight':   '2.0',
                'Grid/MinGroundHeight':     '-0.1',
                'Grid/MaxGroundHeight':     '0.1',
                'Grid/RayTracing':          'true',
                'Grid/FootprintLength':     '1.0',
                'Grid/FootprintWidth':      '0.7',
                'Grid/FootprintHeight':     '0.5',
                'Grid/ClusterRadius':       '0.1',
                'Grid/GroundIsObstacle':    'false',
                'Grid/NormalsSegmentation': 'true',

                'Reg/Strategy':                  '2',
                'Reg/Force3DoF':                 'true',
                'Icp/MaxCorrespondenceDistance': '0.15',
                'Icp/PointToPlane':              'true',
                'Icp/Iterations':                '30',
                'Icp/VoxelSize':                 '0.1',
                'Icp/Epsilon':                   '0.001',
                'Icp/MaxTranslation':            '1.5',
                'Icp/MaxRotation':               '1',
                'Icp/OutlierRatio':              '0.1',

                'Rtabmap/DetectionRate': '1.0',
                'Rtabmap/TimeThr':       '0',
                'Rtabmap/MemoryThr':     '0',
                'wait_for_transform':    2.0,
            }],
            remappings=[
                # ← Output decompressor (bag compressed -> raw), NAMA SAMA
                #   dengan topic live (lihat catatan modul di atas)
                ('rgb/image',       '/camera/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('depth/image',     '/camera/camera/aligned_depth_to_color/image_raw'),
                ('scan_cloud',      '/velodyne_points'),
                ('odom',            odom),
                ('map',             '/map'),
                ('cloud_map',       '/rtabmap/cloud_map'),
                ('grid_map',        '/rtabmap/grid_map'),
            ])

        rtabmap_viz_node = Node(
            package='rtabmap_viz', executable='rtabmap_viz', name='rtabmap_viz',
            output='screen',
            parameters=[{
                'use_sim_time':             True,
                'subscribe_depth':          True,
                'subscribe_rgb':            True,
                'subscribe_scan_cloud':     False,
                'subscribe_odom_info':      False,
                'approx_sync':              True,
                'approx_sync_max_interval': 6.0,
                'sync_queue_size':          50,
                'topic_queue_size':         50,
                'frame_id':                 ROBOT_BASE_FRAME,
                'odom_frame_id':            ODOM_FRAME,
            }],
            remappings=[
                ('rgb/image',       '/camera/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('depth/image',     '/camera/camera/aligned_depth_to_color/image_raw'),
                ('odom',            odom),
            ])

        rviz_node = Node(
            package='rviz2', executable='rviz2', name='rviz2',
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(rviz_flag),
            output='screen')

        mode_str = 'LOCALIZATION' if loc == 'true' else 'MAPPING'
        nodes = [
            LogInfo(msg=f'>>> [T+6s] RTAB-Map {mode_str} mode (replay, '
                        f'Mem/IncrementalMemory={incremental})...'),
            rtabmap_node,
        ]
        if use_viz == 'true':
            nodes.append(rtabmap_viz_node)
        nodes.append(TimerAction(period=2.0, actions=[
            LogInfo(msg='>>> [T+8s] RViz2 (jika rviz:=true)...'),
            rviz_node,
        ]))
        return nodes

    return LaunchDescription([
        declare_bag_path, declare_rate, declare_rviz, declare_rtabmap_viz,
        declare_db_path, declare_use_ekf, declare_odom_topic,
        declare_localization,

        LogInfo(msg='>>> [T+0s] Rosbag play...'),
        rosbag_play,

        TimerAction(period=2.0, actions=[
            LogInfo(msg='>>> [T+2s] Relay TF/odom (replay)...'),
            relay_tf, relay_tf_static, relay_odom, relay_imu,
        ]),

        TimerAction(period=3.0, actions=[
            LogInfo(msg='>>> [T+3s] EKF filter (jika use_ekf=true)...'),
            ekf_node,
        ]),

        TimerAction(period=6.0, actions=[
            OpaqueFunction(function=launch_rtabmap),
        ]),
    ])
