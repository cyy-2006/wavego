"""Isaac Sim 数字孪生后端主脚本。

功能:
  1. 导入 12 自由度四足 URDF, 搭建物理仿真世界
  2. 通过 ROS2 Bridge 发布关节状态  -> /sim/joint_states (sensor_msgs/JointState)
  3. 通过 ROS2 Bridge 订阅关节指令  <- /sim/joint_commands (sensor_msgs/JointState, 位置模式)
  4. 发布仿真时钟 -> /clock

运行方式 (需要先 pip 安装 isaacsim, 见 scripts/install_isaacsim.sh):
  # 方式一: 直接运行 (无需 source ROS 环境)
  python3 src/quadruped_sim/quadruped_sim/isaac_twin.py

  # 方式二: colcon build 之后
  ros2 launch quadruped_sim isaac_sim.launch.py

注意: 建议在【未 source ROS】的终端运行本脚本, Isaac Sim 自带的
ROS 2 humble 内部库会通过 DDS 自动与系统 ROS 2 通信。
"""
import argparse
import os

# ---------------- 参数解析 (必须在 SimulationApp 之前) ----------------
parser = argparse.ArgumentParser(description="Quadruped digital twin - Isaac Sim backend")
parser.add_argument("--urdf", default="", help="quadruped.urdf 路径 (默认自动查找)")
parser.add_argument("--config", default="", help="sim.yaml 配置文件路径 (默认自动查找)")
parser.add_argument("--headless", action="store_true", help="无渲染窗口运行")
ARGS, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402  必须是第一个 isaac 相关 import

simulation_app = SimulationApp({"headless": ARGS.headless, "width": 1280, "height": 720})

import omni.graph.core as og  # noqa: E402
import omni.kit.commands  # noqa: E402

from isaacsim.core import World  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

# 新旧版本 API 命名空间兼容 (4.5 起 omni.isaac.* -> isaacsim.*)
try:
    from isaacsim.core.prims import SingleArticulation
except ImportError:  # Isaac Sim <= 4.2
    from omni.isaac.core.articulations import Articulation as SingleArticulation

try:
    from isaacsim.asset.importer.urdf import _urdf
except ImportError:
    from omni.isaac.urdf import _urdf


# ---------------- 配置加载 ----------------
def _find_file(explicit: str, rel_candidates: list, description: str) -> str:
    """按优先级查找文件: 显式参数 > 脚本相对路径候选。"""
    if explicit:
        if not os.path.isfile(explicit):
            raise FileNotFoundError(f"{description} 不存在: {explicit}")
        return os.path.abspath(explicit)
    here = os.path.dirname(os.path.abspath(__file__))
    for rel in rel_candidates:
        cand = os.path.normpath(os.path.join(here, rel))
        if os.path.isfile(cand):
            return cand
    raise FileNotFoundError(
        f"找不到{description}, 请用参数显式指定. 尝试过: {rel_candidates}"
    )


def load_config():
    cfg_path = _find_file(
        ARGS.config,
        [
            # 源码目录布局
            "../config/sim.yaml",
            # colcon install 布局
            "../../../share/quadruped_sim/config/sim.yaml",
        ],
        "配置文件",
    )
    try:
        import yaml

        with open(cfg_path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)
    except ImportError:
        print("[isaac_twin] 警告: 无 yaml 模块, 使用内置默认配置")
        return {}


def resolve_urdf() -> str:
    return _find_file(
        ARGS.urdf,
        [
            # 源码目录布局: src/quadruped_sim/quadruped_sim/ -> src/quadruped_description/
            "../../quadruped_description/urdf/quadruped.urdf",
            # colcon install 布局
            "../../../../quadruped_description/share/quadruped_description/urdf/quadruped.urdf",
        ],
        "URDF",
    )


# ---------------- URDF 导入 ----------------
def import_robot(urdf_path: str, cfg: dict) -> str:
    import_config = _urdf.ImportConfig()
    import_config.merge_fixed_joints = False
    import_config.fix_base = False            # 四足为浮动基
    import_config.self_collision = False
    import_config.import_inertia_tensor = True
    import_config.convex_decomp = False
    import_config.create_physics_scene = False  # 由 World 统一创建
    import_config.distance_scale = 1.0

    # 关节默认使用位置驱动, 增益从配置读取
    drive = cfg.get("drive", {})
    try:
        import_config.default_drive_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
        import_config.default_position_drive_strength = float(drive.get("stiffness", 40.0))
        import_config.default_position_drive_damping = float(drive.get("damping", 2.0))
    except AttributeError:
        print("[isaac_twin] 警告: 当前 Isaac Sim 版本的驱动增益接口不同, 使用默认值")

    status, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
        dest_path="",
    )
    if not status:
        raise RuntimeError(f"URDF 导入失败: {urdf_path}")
    print(f"[isaac_twin] 机器人已导入: {prim_path}")
    return prim_path


# ---------------- ROS2 Bridge 图 ----------------
def _build_graph_once(ns: str, robot_prim: str, state_topic: str, cmd_topic: str):
    """用指定命名空间前缀构建一次 OmniGraph。ns 为 'isaacsim.ros2.bridge' 等。"""
    core_ns = "isaacsim.core.nodes" if ns.startswith("isaacsim") else "omni.isaac.core_nodes"
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/QuadrupedTwinGraph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", f"{core_ns}.IsaacReadSimulationTime"),
                ("Context", f"{ns}.ROS2Context"),
                ("PubJointStates", f"{ns}.ROS2PublishJointState"),
                ("SubJointCommands", f"{ns}.ROS2SubscribeJointState"),
                ("ArticulationController", f"{core_ns}.IsaacArticulationController"),
                ("PubClock", f"{ns}.ROS2PublishClock"),
            ],
            keys.CONNECT: [
                # 状态发布: tick -> 读关节 -> 发 /sim/joint_states
                ("OnPlaybackTick.outputs:tick", "PubJointStates.inputs:execIn"),
                # 指令订阅: tick -> 收 /sim/joint_commands -> 驱动关节
                ("OnPlaybackTick.outputs:tick", "SubJointCommands.inputs:execIn"),
                ("SubJointCommands.outputs:execOut", "ArticulationController.inputs:execIn"),
                ("SubJointCommands.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                ("SubJointCommands.outputs:positionCommand",
                 "ArticulationController.inputs:positionCommand"),
                ("SubJointCommands.outputs:velocityCommand",
                 "ArticulationController.inputs:velocityCommand"),
                ("SubJointCommands.outputs:effortCommand",
                 "ArticulationController.inputs:effortCommand"),
                # 时钟
                ("OnPlaybackTick.outputs:tick", "PubClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PubClock.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("PubJointStates.inputs:topicName", state_topic),
                ("PubJointStates.inputs:targetPrim", robot_prim),
                ("SubJointCommands.inputs:topicName", cmd_topic),
                ("ArticulationController.inputs:targetPrim", robot_prim),
                ("PubClock.inputs:topicName", "/clock"),
            ],
        },
    )


def create_ros_graph(robot_prim: str, cfg: dict):
    # 启用 ROS2 Bridge 扩展 (新旧命名兼容)
    for ext in ("isaacsim.ros2.bridge", "omni.isaac.ros2_bridge"):
        try:
            enable_extension(ext)
            break
        except Exception:
            continue
    else:
        raise RuntimeError("无法启用 ROS2 Bridge 扩展")

    topics = cfg.get("topics", {})
    state_topic = topics.get("joint_states", "/sim/joint_states")
    cmd_topic = topics.get("joint_commands", "/sim/joint_commands")

    for ns in ("isaacsim.ros2.bridge", "omni.isaac.ros2_bridge"):
        try:
            _build_graph_once(ns, robot_prim, state_topic, cmd_topic)
            print(f"[isaac_twin] ROS2 图已创建 (命名空间 {ns})")
            print(f"[isaac_twin]   发布关节状态 -> {state_topic}")
            print(f"[isaac_twin]   订阅关节指令 <- {cmd_topic}")
            return
        except Exception as e:  # noqa: BLE001
            print(f"[isaac_twin] 命名空间 {ns} 建图失败: {e}, 尝试备选")
    raise RuntimeError("ROS2 建图失败")


# ---------------- 主流程 ----------------
def main():
    cfg = load_config()
    urdf_path = resolve_urdf()
    print(f"[isaac_twin] URDF: {urdf_path}")

    physics_dt = 1.0 / float(cfg.get("physics_rate", 200))
    render_dt = 1.0 / float(cfg.get("render_rate", 60))

    world = World(stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=render_dt)
    world.scene.add_default_ground_plane()

    robot_prim = import_robot(urdf_path, cfg)
    robot = world.scene.add(SingleArticulation(prim_path=robot_prim, name="quadruped"))

    create_ros_graph(robot_prim, cfg)

    world.reset()
    print(f"[isaac_twin] 关节顺序: {robot.dof_names}")
    print("[isaac_twin] 仿真开始, 等待 /sim/joint_commands 指令...")

    while simulation_app.is_running():
        world.step(render=not ARGS.headless)

    simulation_app.close()


if __name__ == "__main__":
    main()
