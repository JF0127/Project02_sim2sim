import math
import time
from typing import Dict, List

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension


class ObsProcessNode(Node):
    def __init__(self) -> None:
        super().__init__('obs_process_node')

        self.joint_state_topic = self.declare_parameter(
            'joint_state_topic', '/robot/joint_state').value
        self.raw_action_topic = self.declare_parameter(
            'raw_action_topic', '/policy/raw_action').value
        self.obs_topic = self.declare_parameter(
            'obs_topic', '/policy/obs').value
        self.controlled_joint_names = self.declare_parameter(
            'controlled_joint_names',
            [
                'left_hip_roll_joint',
                'left_hip_yaw_joint',
                'left_hip_pitch_joint',
                'left_knee_joint',
                'left_ankle_pitch_joint',
                'right_hip_roll_joint',
                'right_hip_yaw_joint',
                'right_hip_pitch_joint',
                'right_knee_joint',
                'right_ankle_pitch_joint',
            ]).value

        self.joint_count = len(self.controlled_joint_names)
        self.last_q: List[float] = [0.0] * self.joint_count
        self.last_dq: List[float] = [0.0] * self.joint_count
        self.last_raw_action: List[float] = [0.0] * self.joint_count

        self.last_warn_time: Dict[str, float] = {}
        self.warn_interval_sec = 2.0

        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            qos_profile_sensor_data,
        )
        self.raw_action_sub = self.create_subscription(
            Float32MultiArray,
            self.raw_action_topic,
            self.raw_action_callback,
            qos_profile_sensor_data,
        )
        self.obs_pub = self.create_publisher(
            Float32MultiArray,
            self.obs_topic,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'obs_process_node started. joints={self.joint_count}, '
            f'joint_state_topic={self.joint_state_topic}, '
            f'raw_action_topic={self.raw_action_topic}, '
            f'obs_topic={self.obs_topic}')

    def warn_throttle(self, key: str, message: str) -> None:
        now = time.monotonic()
        last = self.last_warn_time.get(key, 0.0)
        if now - last >= self.warn_interval_sec:
            self.get_logger().warning(message)
            self.last_warn_time[key] = now

    def _is_valid_number(self, value: float) -> bool:
        return math.isfinite(value)

    def raw_action_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) != self.joint_count:
            self.warn_throttle(
                'raw_action_size',
                f'raw_action size mismatch: got {len(msg.data)}, '
                f'expected {self.joint_count}, dropped.')
            return

        for v in msg.data:
            if not self._is_valid_number(v):
                self.warn_throttle('raw_action_nan', 'raw_action contains NaN/Inf, dropped.')
                return

        self.last_raw_action = [float(v) for v in msg.data]

    def joint_state_callback(self, msg: JointState) -> None:
        name_to_index = {name: idx for idx, name in enumerate(msg.name)}

        for j, joint_name in enumerate(self.controlled_joint_names):
            idx = name_to_index.get(joint_name)
            if idx is None:
                self.warn_throttle(
                    f'missing_joint_{joint_name}',
                    f'joint_state missing joint "{joint_name}", use last value.')
                continue

            if idx < len(msg.position):
                pos = float(msg.position[idx])
                if self._is_valid_number(pos):
                    self.last_q[j] = pos
                else:
                    self.warn_throttle(
                        f'bad_pos_{joint_name}',
                        f'joint "{joint_name}" position is NaN/Inf, use last value.')
            else:
                self.warn_throttle(
                    f'missing_pos_{joint_name}',
                    f'joint_state missing position for "{joint_name}", use last value.')

            if idx < len(msg.velocity):
                vel = float(msg.velocity[idx])
                if self._is_valid_number(vel):
                    self.last_dq[j] = vel
                else:
                    self.warn_throttle(
                        f'bad_vel_{joint_name}',
                        f'joint "{joint_name}" velocity is NaN/Inf, use last value.')
            else:
                self.warn_throttle(
                    f'missing_vel_{joint_name}',
                    f'joint_state missing velocity for "{joint_name}", use last value.')

        obs = self.last_q + self.last_dq + self.last_raw_action
        obs_msg = Float32MultiArray()
        obs_msg.layout.dim = [
            MultiArrayDimension(
                label='obs',
                size=len(obs),
                stride=len(obs),
            )
        ]
        obs_msg.data = [float(v) for v in obs]
        self.obs_pub.publish(obs_msg)


def main() -> None:
    rclpy.init()
    node = ObsProcessNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
