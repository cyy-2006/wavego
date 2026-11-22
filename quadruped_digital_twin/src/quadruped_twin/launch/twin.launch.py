"""启动数字孪生同步桥。

用法:
  ros2 launch quadruped_twin twin.launch.py              # sim_only
  ros2 launch quadruped_twin twin.launch.py mode:=mirror_real
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode_arg = DeclareLaunchArgument('mode', default_value='sim_only')

    bridge = Node(
        package='quadruped_twin',
        executable='twin_bridge',
        parameters=[{'mode': LaunchConfiguration('mode')}],
        output='screen',
    )

    return LaunchDescription([mode_arg, bridge])
