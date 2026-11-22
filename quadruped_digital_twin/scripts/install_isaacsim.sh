#!/usr/bin/env bash
# 安装 Isaac Sim (pip 方式, 官方支持 Python 3.10 / Linux)
# 参考: https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/pip_installation.html
#
# 体积较大 (含扩展缓存约 10GB+), 安装后运行验证:
#   python3 -c "from isaacsim import SimulationApp; print('ok')"
set -e

# 带扩展缓存, 首次启动不用在线下载扩展
pip install "isaacsim[all,extscache]==4.5.0" --extra-index-url https://pypi.nvidia.com

echo "安装完成。运行数字孪生后端:"
echo "  python3 src/quadruped_sim/quadruped_sim/isaac_twin.py"
