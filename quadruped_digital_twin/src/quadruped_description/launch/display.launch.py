"""RViz 中查看 12 自由度四足模型(纯展示, 不需要 Isaac Sim)。

用法:
    ros2 launch quadruped_description display.launch.py            # 带关节滑条
    ros2 launch quadruped_description display.launch.py gui:=false # 无滑条(零位)

注意: gui:=true 需要 joint_state_publisher_gui 包:
    sudo apt install ros-humble-joint-state-publisher-gui
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('quadruped_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'quadruped.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'quadruped.rviz')

    gui_arg = DeclareLaunchArgument('gui', default_value='true')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]), value_type=str)}],
    )

    joint_state_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([gui_arg, robot_state_publisher, joint_state_gui, rviz])
