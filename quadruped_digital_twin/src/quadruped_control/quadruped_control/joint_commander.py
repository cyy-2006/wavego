"""关节指令发布节点: 向仿真后端发送 12 关节位置指令。

话题: /sim/joint_commands (sensor_msgs/JointState, 仅使用 name + position 字段)

模式 (参数 mode):
  stand : 固定站立姿态
  sine  : 对角步态式摆动演示 (验证指令链路, 非真实步态)
  hold  : 保持零位

运行:
  ros2 run quadruped_control joint_commander --ros-args -p mode:=stand
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

# 关节顺序与 URDF 一致
JOINT_NAMES = [
    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
]

# 站立姿态 (rad)。符号约定见 URDF: 大腿前摆为正(绕 +Y 使足端向后),
# 膝关节弯曲为正。具体数值需在仿真中微调。
STAND_POSE = {
    'hip': 0.0,
    'thigh': -0.4,
    'calf': 0.8,
}


class JointCommander(Node):
    def __init__(self):
        super().__init__('joint_commander')

        self.declare_parameter('mode', 'stand')          # stand | sine | hold
        self.declare_parameter('rate', 50.0)             # 发布频率 Hz
        self.declare_parameter('topic', '/sim/joint_commands')
        self.declare_parameter('sine_amplitude', 0.3)    # 大腿摆动幅值 rad
        self.declare_parameter('sine_frequency', 1.0)    # 摆动频率 Hz

        self.mode = self.get_parameter('mode').value
        self.amplitude = self.get_parameter('sine_amplitude').value
        self.frequency = self.get_parameter('sine_frequency').value
        rate = self.get_parameter('rate').value
        topic = self.get_parameter('topic').value

        self.pub = self.create_publisher(JointState, topic, 10)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / rate, self.publish_commands)

        self.get_logger().info(
            f'关节指令节点已启动: mode={self.mode}, {rate} Hz -> {topic}')

    def _stand_positions(self):
        """12 关节站立姿态, 顺序与 JOINT_NAMES 一致。"""
        positions = []
        for name in JOINT_NAMES:
            if 'hip_joint' in name:
                positions.append(STAND_POSE['hip'])
            elif 'thigh_joint' in name:
                positions.append(STAND_POSE['thigh'])
            else:
                positions.append(STAND_POSE['calf'])
        return positions

    def _sine_positions(self, t: float):
        """对角腿同相的摆动演示: FL+RR 一组, FR+RL 一组, 相位差 pi。"""
        positions = self._stand_positions()
        w = 2.0 * math.pi * self.frequency
        for i, name in enumerate(JOINT_NAMES):
            leg = name.split('_')[0]
            phase = 0.0 if leg in ('FL', 'RR') else math.pi
            if 'thigh_joint' in name:
                positions[i] += self.amplitude * math.sin(w * t + phase)
            elif 'calf_joint' in name:
                positions[i] += 0.5 * self.amplitude * math.sin(w * t + phase + math.pi / 2)
        return positions

    def publish_commands(self):
        t = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        if self.mode == 'stand':
            positions = self._stand_positions()
        elif self.mode == 'sine':
            positions = self._sine_positions(t)
        else:  # hold
            positions = [0.0] * len(JOINT_NAMES)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = positions
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointCommander()
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
