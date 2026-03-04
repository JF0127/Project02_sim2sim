import math
import time
from typing import Dict, List

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension


class RlPolicyNode(Node):
    def __init__(self) -> None:
        super().__init__('rl_policy_node')

        self.obs_topic = self.declare_parameter('obs_topic', '/policy/obs').value
        self.raw_action_topic = self.declare_parameter(
            'raw_action_topic', '/policy/raw_action').value
        self.action_dim = int(self.declare_parameter('action_dim', 10).value)
        self.obs_dim_expected = int(self.declare_parameter('obs_dim_expected', 30).value)
        self.policy_mode = self.declare_parameter('policy_mode', 'copy_last_action').value
        self.action_scale = float(self.declare_parameter('action_scale', 1.0).value)

        if self.action_dim <= 0:
            raise ValueError('action_dim must be > 0')
        if self.obs_dim_expected < 0:
            raise ValueError('obs_dim_expected must be >= 0')

        self.last_action: List[float] = [0.0] * self.action_dim
        self.last_warn_time: Dict[str, float] = {}
        self.warn_interval_sec = 2.0

        self.obs_sub = self.create_subscription(
            Float32MultiArray,
            self.obs_topic,
            self.obs_callback,
            qos_profile_sensor_data,
        )
        self.raw_action_pub = self.create_publisher(
            Float32MultiArray,
            self.raw_action_topic,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'rl_policy_node started. obs_topic={self.obs_topic}, '
            f'raw_action_topic={self.raw_action_topic}, action_dim={self.action_dim}, '
            f'obs_dim_expected={self.obs_dim_expected}, policy_mode={self.policy_mode}')

    def warn_throttle(self, key: str, message: str) -> None:
        now = time.monotonic()
        last = self.last_warn_time.get(key, 0.0)
        if now - last >= self.warn_interval_sec:
            self.get_logger().warning(message)
            self.last_warn_time[key] = now

    def _is_valid_number(self, value: float) -> bool:
        return math.isfinite(value)

    def infer(self, obs: List[float]) -> List[float]:
        if self.policy_mode == 'zero':
            action = [0.0] * self.action_dim
        elif self.policy_mode == 'tanh_head':
            action = [0.0] * self.action_dim
            for i in range(self.action_dim):
                if i < len(obs):
                    action[i] = math.tanh(obs[i])
        elif self.policy_mode == 'copy_last_action':
            if len(obs) >= self.action_dim:
                action = [float(v) for v in obs[-self.action_dim:]]
            else:
                self.warn_throttle(
                    'obs_too_short_copy_last_action',
                    f'obs length {len(obs)} < action_dim {self.action_dim}, hold last action.')
                action = list(self.last_action)
        else:
            self.warn_throttle(
                'unknown_policy_mode',
                f'Unknown policy_mode="{self.policy_mode}", fallback to zero.')
            action = [0.0] * self.action_dim

        if self.action_scale != 1.0:
            action = [self.action_scale * v for v in action]

        return action

    def obs_callback(self, msg: Float32MultiArray) -> None:
        obs = [float(v) for v in msg.data]
        if self.obs_dim_expected > 0 and len(obs) != self.obs_dim_expected:
            self.warn_throttle(
                'obs_dim_mismatch',
                f'obs dim mismatch: got {len(obs)}, expected {self.obs_dim_expected}.')

        for v in obs:
            if not self._is_valid_number(v):
                self.warn_throttle('obs_nan', 'obs contains NaN/Inf, hold last action.')
                self.publish_action(self.last_action)
                return

        action = self.infer(obs)
        for v in action:
            if not self._is_valid_number(v):
                self.warn_throttle('action_nan', 'inferred action contains NaN/Inf, hold last action.')
                self.publish_action(self.last_action)
                return

        self.last_action = list(action)
        self.publish_action(action)

    def publish_action(self, action: List[float]) -> None:
        msg = Float32MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(
                label='raw_action',
                size=len(action),
                stride=len(action),
            )
        ]
        msg.data = [float(v) for v in action]
        self.raw_action_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = RlPolicyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
