#!/usr/bin/env python3

import copy

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image


DEPTH_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class K1DepthReencodeNode(Node):
    def __init__(self) -> None:
        super().__init__("k1_depth_reencode")

        self.declare_parameter("input_topic", "/StereoNetNode/stereonet_depth")
        self.declare_parameter("output_topic", "/k1/depth_image_16uc1")

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._warned_encodings: set[str] = set()

        self._publisher = self.create_publisher(Image, self._output_topic, DEPTH_QOS)
        self.create_subscription(Image, self._input_topic, self._handle_image, DEPTH_QOS)

        self.get_logger().info(
            "Re-encoding %s -> %s for depthimage_to_laserscan compatibility"
            % (self._input_topic, self._output_topic)
        )

    def _handle_image(self, msg: Image) -> None:
        normalized = msg.encoding.strip().lower()
        output = copy.copy(msg)

        if normalized == "mono16":
            output.encoding = "16UC1"
        elif normalized in {"16uc1", "32fc1"}:
            output.encoding = msg.encoding.upper() if normalized == "16uc1" else "32FC1"
        else:
            if normalized not in self._warned_encodings:
                self.get_logger().warn(
                    "Unexpected depth encoding %r on %s; forwarding unchanged"
                    % (msg.encoding, self._input_topic)
                )
                self._warned_encodings.add(normalized)
        self._publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = K1DepthReencodeNode()
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
