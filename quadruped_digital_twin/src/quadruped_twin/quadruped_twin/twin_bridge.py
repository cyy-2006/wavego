"""数字孪生同步桥: 管理 "物理机器人 <-> 仿真镜像" 的状态同步。

模式 (参数 mode):
  sim_only    : 纯仿真。/sim/joint_states -> /joint_states (供 RViz/下游使用)
  mirror_real : 孪生镜像。/real/joint_states -> /sim/joint_commands,
                虚拟机器人跟随真实机器人姿态; 同时计算虚实同步误差
                -> /twin/sync_error (std_msgs/Float64, 关节最大绝对误差 rad)

话题约定:
  /real/joint_states   真实机器人驱动器发布 (接入实物时使用)
  /sim/joint_states    Isaac Sim 仿真发布
  /sim/joint_commands  发给仿真的位置指令
  /joint_states        对外统一的机器人状态 (RViz 等使用)

运行:
  ros2 run quadruped_twin twin_bridge --ros-args -p mode:=sim_only
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class TwinBridge(Node):
    def __init__(self):
        super().__init__('twin_bridge')

        self.declare_parameter('mode', 'sim_only')   # sim_only | mirror_real
        self.declare_parameter('sim_state_topic', '/sim/joint_states')
        self.declare_parameter('sim_cmd_topic', '/sim/joint_commands')
        self.declare_parameter('real_state_topic', '/real/joint_states')
        self.declare_parameter('out_state_topic', '/joint_states')

        self.mode = self.get_parameter('mode').value
        if self.mode not in ('sim_only', 'mirror_real'):
            raise ValueError(f'未知 mode: {self.mode}')

        sim_state_topic = self.get_parameter('sim_state_topic').value
        sim_cmd_topic = self.get_parameter('sim_cmd_topic').value
        real_state_topic = self.get_parameter('real_state_topic').value
        out_state_topic = self.get_parameter('out_state_topic').value

        # 对外统一状态
        self.state_pub = self.create_publisher(JointState, out_state_topic, 10)
        # 仿真端状态 (镜像模式下用于计算同步误差)
        self.latest_sim_state = None
        self.create_subscription(JointState, sim_state_topic,
                                 self._on_sim_state, 10)

        if self.mode == 'mirror_real':
            self.cmd_pub = self.create_publisher(JointState, sim_cmd_topic, 10)
            self.err_pub = self.create_publisher(Float64, '/twin/sync_error', 10)
            self.create_subscription(JointState, real_state_topic,
                                     self._on_real_state, 10)

        self.get_logger().info(
            f'数字孪生同步桥已启动: mode={self.mode}')

    def _on_sim_state(self, msg: JointState):
        self.latest_sim_state = msg
        if self.mode == 'sim_only':
            # 纯仿真: 直接转发为对外状态
            self.state_pub.publish(msg)

    def _on_real_state(self, msg: JointState):
        # 1) 真实姿态 -> 仿真指令, 虚拟机器人跟随实物
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = list(msg.name)
        cmd.position = list(msg.position)
        self.cmd_pub.publish(cmd)

        # 2) 对外状态以真实机器人为准
        self.state_pub.publish(msg)

        # 3) 虚实同步误差 (按关节名对齐, 取最大绝对误差)
        if self.latest_sim_state is not None:
            sim_pos = dict(zip(self.latest_sim_state.name,
                               self.latest_sim_state.position))
            errors = [abs(p - sim_pos[n])
                      for n, p in zip(msg.name, msg.position) if n in sim_pos]
            if errors:
                self.err_pub.publish(Float64(data=max(errors)))


def main(args=None):
    rclpy.init(args=args)
    node = TwinBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
