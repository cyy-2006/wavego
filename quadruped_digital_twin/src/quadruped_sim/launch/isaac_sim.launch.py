"""启动 Isaac Sim 数字孪生后端。

等价于直接运行 `python3 -m quadruped_sim.isaac_twin`, 此处仅做封装。
注意: 需要已 pip 安装 isaacsim; 若运行环境 source 了 ROS 导致库冲突,
建议改为在未 source ROS 的终端直接运行脚本。
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    sim_share = get_package_share_directory('quadruped_sim')
    desc_share = get_package_share_directory('quadruped_description')

    headless_arg = DeclareLaunchArgument('headless', default_value='false')

    config = os.path.join(sim_share, 'config', 'sim.yaml')
    urdf = os.path.join(desc_share, 'urdf', 'quadruped.urdf')

    # headless:=true 时追加 --headless; 空串会被脚本的 parse_known_args 忽略
    headless_flag = PythonExpression(
        ["'--headless' if '", LaunchConfiguration('headless'), "' == 'true' else ''"]
    )

    isaac_proc = ExecuteProcess(
        cmd=[
            sys.executable, '-m', 'quadruped_sim.isaac_twin',
            '--config', config,
            '--urdf', urdf,
            headless_flag,
        ],
        output='screen',
    )

    return LaunchDescription([headless_arg, isaac_proc])
