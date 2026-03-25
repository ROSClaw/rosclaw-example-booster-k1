#!/usr/bin/env python3

from collections import defaultdict

import rclpy
from booster_interface.msg import LowCmd, LowState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import JointState


STATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

COMMAND_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class K1LowLevelRelay(Node):
    def __init__(self) -> None:
        super().__init__("k1_low_level_relay")

        self.declare_parameter("relay_namespace", "/k1")
        relay_namespace = self.get_parameter("relay_namespace").get_parameter_value().string_value
        self._relay_namespace = self._normalize_namespace(relay_namespace)
        self._counts = defaultdict(int)

        self._low_state_pub = self.create_publisher(
            LowState, self._topic_in_namespace("low_state"), STATE_QOS
        )
        self._joint_states_pub = self.create_publisher(
            JointState, self._topic_in_namespace("joint_states"), STATE_QOS
        )
        self._joint_ctrl_pub = self.create_publisher(LowCmd, "/joint_ctrl", COMMAND_QOS)

        self.create_subscription(LowState, "/low_state", self._relay_low_state, STATE_QOS)
        self.create_subscription(
            JointState, "/joint_states", self._relay_joint_states, STATE_QOS
        )
        self.create_subscription(
            LowCmd, self._topic_in_namespace("joint_ctrl"), self._relay_joint_ctrl, COMMAND_QOS
        )
        self.create_subscription(
            LowCmd, self._topic_in_namespace("low_cmd"), self._relay_joint_ctrl, COMMAND_QOS
        )

        self.create_timer(10.0, self._log_status)

        self.get_logger().info(
            "Relaying /low_state -> %s, /joint_states -> %s, %s and %s -> /joint_ctrl"
            % (
                self._topic_in_namespace("low_state"),
                self._topic_in_namespace("joint_states"),
                self._topic_in_namespace("joint_ctrl"),
                self._topic_in_namespace("low_cmd"),
            )
        )

    def _normalize_namespace(self, namespace: str) -> str:
        stripped = namespace.strip()
        if not stripped:
            return "/k1"
        if not stripped.startswith("/"):
            stripped = "/" + stripped
        return stripped.rstrip("/")

    def _topic_in_namespace(self, suffix: str) -> str:
        return f"{self._relay_namespace}/{suffix}"

    def _relay_low_state(self, msg: LowState) -> None:
        self._low_state_pub.publish(msg)
        self._counts["low_state"] += 1

    def _relay_joint_states(self, msg: JointState) -> None:
        self._joint_states_pub.publish(msg)
        self._counts["joint_states"] += 1

    def _relay_joint_ctrl(self, msg: LowCmd) -> None:
        self._joint_ctrl_pub.publish(msg)
        self._counts["joint_ctrl"] += 1

    def _log_status(self) -> None:
        self.get_logger().info(
            "relay counts low_state=%d joint_states=%d joint_ctrl=%d"
            % (
                self._counts["low_state"],
                self._counts["joint_states"],
                self._counts["joint_ctrl"],
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = K1LowLevelRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
