#!/usr/bin/env python3

import math

import rclpy
from booster_interface.msg import Odometer
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from tf2_ros import TransformBroadcaster


ODOMETER_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


def quaternion_from_yaw(yaw: float) -> Quaternion:
    msg = Quaternion()
    msg.z = math.sin(yaw * 0.5)
    msg.w = math.cos(yaw * 0.5)
    return msg


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class K1OdometerBridge(Node):
    def __init__(self) -> None:
        super().__init__("k1_odometer_bridge")

        self.declare_parameter("input_topic", "/k1/odometer_state")
        self.declare_parameter("output_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "trunk_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("position_scale", 0.01)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._publish_tf = bool(self.get_parameter("publish_tf").value)
        self._position_scale = float(self.get_parameter("position_scale").value)

        self._odom_pub = self.create_publisher(Odometry, self._output_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None

        self._last_x: float | None = None
        self._last_y: float | None = None
        self._last_theta: float | None = None
        self._last_stamp_ns: int | None = None

        self.create_subscription(Odometer, self._input_topic, self._handle_odometer, ODOMETER_QOS)

        self.get_logger().info(
            "Bridging %s -> %s with TF %s -> %s (position_scale=%.4f)"
            % (
                self._input_topic,
                self._output_topic,
                self._odom_frame,
                self._base_frame,
                self._position_scale,
            )
        )

    def _handle_odometer(self, msg: Odometer) -> None:
        now = self.get_clock().now()
        stamp = now.to_msg()
        stamp_ns = now.nanoseconds

        x = float(msg.x) * self._position_scale
        y = float(msg.y) * self._position_scale
        theta = float(msg.theta)

        linear_x = 0.0
        linear_y = 0.0
        angular_z = 0.0

        if self._last_stamp_ns is not None:
            dt = max((stamp_ns - self._last_stamp_ns) / 1e9, 1e-6)
            dx_world = x - float(self._last_x)
            dy_world = y - float(self._last_y)
            dtheta = normalize_angle(theta - float(self._last_theta))

            cos_yaw = math.cos(theta)
            sin_yaw = math.sin(theta)
            linear_x = (cos_yaw * dx_world + sin_yaw * dy_world) / dt
            linear_y = (-sin_yaw * dx_world + cos_yaw * dy_world) / dt
            angular_z = dtheta / dt

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion_from_yaw(theta)
        odom.twist.twist.linear = Vector3(x=linear_x, y=linear_y, z=0.0)
        odom.twist.twist.angular = Vector3(x=0.0, y=0.0, z=angular_z)
        self._odom_pub.publish(odom)

        if self._tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self._odom_frame
            transform.child_frame_id = self._base_frame
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = 0.0
            transform.transform.rotation = quaternion_from_yaw(theta)
            self._tf_broadcaster.sendTransform(transform)

        self._last_x = x
        self._last_y = y
        self._last_theta = theta
        self._last_stamp_ns = stamp_ns


def main(args=None) -> None:
    rclpy.init(args=args)
    node = K1OdometerBridge()
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
