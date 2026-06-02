"""
rtabmap_live.launch.py
Live mode untuk RTABMap di AGX Orin.

Perubahan dari rtabmap_rosbag.launch.py:
  1. Hapus rosbag_play (live mode, sensor langsung)
  2. use_sim_time = False (pakai real time, bukan clock dari bag)
  3. Hapus decompress_color & decompress_depth
     (RealSense langsung publish uncompressed Image)
  4. Topic remapping ke topic RealSense live:
     - /camera/camera/color/image_raw
     - /camera/camera/aligned_depth_to_color/image_raw
     - /camera/camera/color/camera_info
  5. EKF default true (live butuh EKF kalau pakai /a200_1060/platform/odom)
     atau pakai /a200_1060/platform/odom/filtered langsung tanpa EKF

PRASYARAT (sebelum jalankan launch ini):
  - RealSense D455 driver sudah aktif
    ros2 launch realsense2_camera rs_launch.py \\
        enable_color:=true enable_depth:=true align_depth.enable:=true
  - Velodyne VLP-32C driver aktif (publish /velodyne_points)
  - Husky platform aktif (publish /a200_1060/platform/odom/filtered + tf)
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, TimerAction, LogInfo, OpaqueFunction
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOT_BASE_FRAME = 'a200_1060/base_link'
ODOM_FRAME       = 'odom'
MAP_FRAME        = 'map'


def generate_launch_description():

    rviz         = LaunchConfiguration('rviz')
    db_path      = LaunchConfiguration('db_path')
    use_ekf      = LaunchConfiguration('use_ekf')
    odom_topic   = LaunchConfiguration('odom_topic')
    localization = LaunchConfiguration('localization')

    # ── Argumen ────────────────────────────────────────────────────
    declare_rviz       = DeclareLaunchArgument('rviz', default_value='true')
    declare_db_path    = DeclareLaunchArgument(
        'db_path', default_value='/home/kmp-orin/.ros/rtabmap.db')
    declare_use_ekf    = DeclareLaunchArgument(
        'use_ekf', default_value='false',
        description='false=pakai /a200_1060/platform/odom/filtered langsung | true=EKF custom')
    declare_odom_topic = DeclareLaunchArgument(
        'odom_topic', default_value='/a200_1060/platform/odom/filtered',
        description='Topic odom ke RTAB-Map')
    declare_localization = DeclareLaunchArgument(
        'localization', default_value='false',
        description='true=localization mode | false=mapping mode')

    # ══════════════════════════════════════════════════════════════
    # T+0s — RELAY TF + ODOM (bridge namespace a200_1060)
    # ══════════════════════════════════════════════════════════════
    relay_tf = Node(
        package='topic_tools', executable='relay', name='relay_tf',
        arguments=['/a200_1060/tf', '/tf'],
        parameters=[{'use_sim_time': False}], output='screen')

    relay_tf_static = Node(
        package='topic_tools', executable='relay', name='relay_tf_static',
        arguments=['/a200_1060/tf_static', '/tf_static'],
        parameters=[{'use_sim_time': False}], output='screen')

    relay_odom = Node(
        package='topic_tools', executable='relay', name='relay_odom',
        arguments=['/a200_1060/platform/odom', '/odom'],
        parameters=[{'use_sim_time': False}], output='screen')

    relay_imu = Node(
        package='topic_tools', executable='relay', name='relay_imu',
        arguments=['/a200_1060/sensors/imu_0/data', '/imu/data'],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_ekf),
        output='screen')

    # ══════════════════════════════════════════════════════════════
    # T+0s — STATIC TF (kamera ke base_link)
    # ══════════════════════════════════════════════════════════════
    tf_base_camera = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_base_camera',
        arguments=['0.2', '0', '0.25', '0', '0', '0',
                   'a200_1060/base_link', 'camera_link'],
        parameters=[{'use_sim_time': False}], output='screen')

    tf_camera_color_optical = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_camera_color_optical',
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708',
                   'camera_link', 'camera_color_optical_frame'],
        parameters=[{'use_sim_time': False}], output='screen')

    tf_camera_depth_optical = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_camera_depth_optical',
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708',
                   'camera_link', 'camera_depth_optical_frame'],
        parameters=[{'use_sim_time': False}], output='screen')

    tf_bridge_base = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_bridge_base',
        arguments=['0', '0', '0', '0', '0', '0',
                   'base_link', 'a200_1060/base_link'],
        parameters=[{'use_sim_time': False}], output='screen')

    # ══════════════════════════════════════════════════════════════
    # T+1s — EKF NODE (opsional)
    # ══════════════════════════════════════════════════════════════
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=IfCondition(use_ekf),
        parameters=[{
            'use_sim_time':     False,
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
    # T+4s — RTAB-MAP (via OpaqueFunction)
    # ══════════════════════════════════════════════════════════════
    def launch_rtabmap(context, *args, **kwargs):
        loc       = context.launch_configurations.get('localization', 'false')
        db        = context.launch_configurations.get('db_path', '/home/kmp-orin/.ros/rtabmap.db')
        odom      = context.launch_configurations.get('odom_topic',
                        '/a200_1060/platform/odom/filtered')
        rviz_flag = context.launch_configurations.get('rviz', 'true')
        incremental = 'false' if loc == 'true' else 'true'

        rtabmap_node = Node(
            package='rtabmap_slam', executable='rtabmap', name='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time':         False,   # ← LIVE MODE
                'database_path':        db,
                'frame_id':             ROBOT_BASE_FRAME,
                'map_frame_id':         MAP_FRAME,
                'odom_frame_id':        ODOM_FRAME,
                'subscribe_depth':      True,
                'subscribe_rgb':        True,
                'subscribe_scan_cloud': True,
                'subscribe_odom_info':  False,
                'approx_sync':              True,
                'approx_sync_max_interval': 1.0,
                'sync_queue_size':          50,
                'topic_queue_size':         50,

                'Mem/STMSize':             '30',
                'Mem/RehearsalSimilarity': '0.45',
                'Mem/NotLinkedNodesKept':  'false',
                'Mem/IncrementalMemory':   incremental,
                'Mem/InitWMWithAllNodes':  'false',

                'Kp/DetectorStrategy': '6',
                'Vis/FeatureType':     '6',
                'Kp/MaxFeatures':      '600',
                'Vis/MinInliers':      '10',
                'Vis/InlierDistance':  '0.1',
                'Vis/EstimationType':  '1',
                'Vis/MaxDepth':        '10.0',

                'RGBD/OptimizeFromGraphEnd': 'false',
                'RGBD/OptimizeMaxError':     '4.0',
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
                'Grid/FootprintLength':     '0.8',
                'Grid/FootprintWidth':      '0.5',
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
            }],
            remappings=[
                # ← Topic RealSense live (bukan dari decompressor)
                ('rgb/image',       '/camera/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('depth/image',     '/camera/camera/depth/image_rect_raw'),
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
                'use_sim_time':             False,
                'subscribe_depth':          True,
                'subscribe_rgb':            True,
                'subscribe_scan_cloud':     False,
                'subscribe_odom_info':      False,
                'approx_sync':              True,
                'approx_sync_max_interval': 2.0,
                'sync_queue_size':          50,
                'topic_queue_size':         50,
                'frame_id':                 ROBOT_BASE_FRAME,
                'odom_frame_id':            ODOM_FRAME,
            }],
            remappings=[
                ('rgb/image',       '/camera/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('depth/image',     '/camera/camera/depth/image_rect_raw'),
                ('odom',            odom),
            ])

        rviz_node = Node(
            package='rviz2', executable='rviz2', name='rviz2',
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(rviz_flag),
            output='screen')

        mode_str = 'LOCALIZATION' if loc == 'true' else 'MAPPING'
        return [
            LogInfo(msg=f'>>> [T+4s] RTAB-Map {mode_str} mode '
                        f'(Mem/IncrementalMemory={incremental})...'),
            rtabmap_node,
            rtabmap_viz_node,
            TimerAction(period=2.0, actions=[
                LogInfo(msg='>>> [T+6s] RViz2...'),
                rviz_node,
            ]),
        ]

    return LaunchDescription([
        declare_rviz, declare_db_path,
        declare_use_ekf, declare_odom_topic,
        declare_localization,

        LogInfo(msg='>>> [T+0s] Relay TF + Static TF (live mode, no rosbag)...'),
        relay_tf, relay_tf_static, relay_odom, relay_imu,
        tf_base_camera, tf_camera_color_optical,
        tf_camera_depth_optical, tf_bridge_base,

        TimerAction(period=1.0, actions=[
            LogInfo(msg='>>> [T+1s] EKF filter (jika use_ekf=true)...'),
            ekf_node,
        ]),

        TimerAction(period=4.0, actions=[
            OpaqueFunction(function=launch_rtabmap),
        ]),
    ])
