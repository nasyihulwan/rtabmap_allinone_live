import os
"""
rtabmap_mapping_only.launch.py
Mapping ringan tanpa semantic nodes (YOLO/SegFormer).
Cocok untuk sesi mapping mandiri atau playback rosbag.

Parameter baru: use_amcl
  false (default) — RTABMap publish map→odom TF seperti biasa
  true            — RTABMap tidak publish TF (AMCL yang handle, hindari konflik)
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
    declare_rviz = DeclareLaunchArgument('rviz', default_value='true')
    declare_rtabmap_viz = DeclareLaunchArgument(
        'use_rtabmap_viz', default_value='false',
        description='true=launch rtabmap_viz')
    declare_db_path = DeclareLaunchArgument(
        'db_path', default_value=os.path.expanduser('~/.ros/rtabmap.db'))
    declare_use_ekf = DeclareLaunchArgument(
        'use_ekf', default_value='false',
        description='false=pakai /a200_1060/platform/odom/filtered langsung | true=EKF custom')
    declare_odom_topic = DeclareLaunchArgument(
        'odom_topic', default_value='/a200_1060/platform/odom/filtered',
        description='Topic odom ke RTAB-Map')
    declare_localization = DeclareLaunchArgument(
        'localization', default_value='false',
        description='true=localization mode | false=mapping mode')
    declare_use_amcl = DeclareLaunchArgument(
        'use_amcl', default_value='false',
        description='true=AMCL handle map->odom TF | false=RTABMap handle')

    # ══════════════════════════════════════════════════════════════
    # T+0s — RELAY TF + ODOM (bridge namespace a200_1060)
    # ══════════════════════════════════════════════════════════════
    relay_tf = Node(
        package='topic_tools', executable='relay', name='relay_tf',
        arguments=['/a200_1060/tf', '/tf'],
        parameters=[{'use_sim_time': False}], output='log')

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

    # Camera/velodyne/base_link static TF: lihat static_tf_live.launch.py
    # (run.sh selalu menjalankannya bareng launch file ini).

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
        loc             = context.launch_configurations.get('localization', 'false')
        db              = context.launch_configurations.get('db_path', os.path.expanduser('~/.ros/rtabmap.db'))
        odom            = context.launch_configurations.get('odom_topic',
                              '/a200_1060/platform/odom/filtered')
        rviz_flag       = context.launch_configurations.get('rviz', 'true')
        use_rtabmap_viz = context.launch_configurations.get('use_rtabmap_viz', 'false')
        use_amcl        = context.launch_configurations.get('use_amcl', 'false')
        incremental     = 'false' if loc == 'true' else 'true'
        publish_tf      = False if use_amcl == 'true' else True

        rtabmap_node = Node(
            package='rtabmap_slam', executable='rtabmap', name='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time':         False,
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
                'publish_tf':               publish_tf,

                'Mem/STMSize':             '30',
                'Mem/RehearsalSimilarity': '0.45',
                'Mem/NotLinkedNodesKept':  'false',
                'Mem/IncrementalMemory':   incremental,
                'Mem/InitWMWithAllNodes':  'false',

                'Kp/DetectorStrategy': '6',
                'Vis/FeatureType':     '6',
                'Kp/MaxFeatures':      '500',
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
                'wait_for_transform':    0.5,
            }],
            remappings=[
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

        mode_str  = 'LOCALIZATION' if loc == 'true' else 'MAPPING'
        amcl_str  = ' [AMCL-TF]' if use_amcl == 'true' else ''
        nodes = [
            LogInfo(msg=f'>>> [T+4s] RTAB-Map {mode_str} mode{amcl_str} '
                        f'(Mem/IncrementalMemory={incremental}, publish_tf={publish_tf})...'),
            rtabmap_node,
        ]
        if use_rtabmap_viz == 'true':
            nodes.append(rtabmap_viz_node)
        nodes.append(TimerAction(period=2.0, actions=[
            LogInfo(msg='>>> [T+6s] RViz2...'),
            rviz_node,
        ]))
        return nodes

    return LaunchDescription([
        declare_rviz, declare_rtabmap_viz, declare_db_path,
        declare_use_ekf, declare_odom_topic,
        declare_localization, declare_use_amcl,

        LogInfo(msg='>>> [T+0s] Relay TF (mapping only mode)...'),
        relay_tf, relay_tf_static, relay_odom, relay_imu,

        TimerAction(period=1.0, actions=[
            LogInfo(msg='>>> [T+1s] EKF filter (jika use_ekf=true)...'),
            ekf_node,
        ]),

        TimerAction(period=4.0, actions=[
            OpaqueFunction(function=launch_rtabmap),
        ]),
    ])
