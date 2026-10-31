"""visualizer_qt.launch.py — real ROS2-wired Qt/QML onboard visualizer.

repo_root (PRD §5.9) has no safe default — it's the absolute path to the
rtabmap_allinone_live checkout on whichever machine this runs on (Orin path
differs from this dev machine), needed so the Start-sensors/Start-pipeline
buttons can find start_sensors.sh/run.sh. Leave empty to disable that
feature (the app still works as a pure viewer); set it explicitly when
deploying, e.g.:
  ros2 launch visualizer_qt visualizer_qt.launch.py repo_root:=/home/ubuntu/rtabmap_allinone_live
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use /clock from a bag instead of wall-clock time.')
    declare_repo_root = DeclareLaunchArgument(
        'repo_root', default_value='',
        description='Absolute path to the rtabmap_allinone_live checkout '
                     '(PRD §5.9 start-sensors/start-pipeline buttons).')

    visualizer_qt_node = Node(
        package='visualizer_qt',
        executable='visualizer_qt_node',
        name='visualizer_qt_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'repo_root': LaunchConfiguration('repo_root'),
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_repo_root,
        visualizer_qt_node,
    ])
