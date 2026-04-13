#!/usr/bin/env python3

import json
import math
from typing import Optional

import rclpy
from booster_interface.srv import RpcService
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


MOVE_API_ID = 2001
EPSILON = 1e-4


class K1CmdVelBridge(Node):
    def __init__(self) -> None:
        super().__init__("k1_cmd_vel_bridge")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("rpc_service_name", "/booster_rpc_service")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("command_timeout_sec", 1.0)
        self.declare_parameter("linear_x_max", 0.5)
        self.declare_parameter("linear_y_max", 0.3)
        self.declare_parameter("angular_z_max", 1.0)

        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._rpc_service_name = str(self.get_parameter("rpc_service_name").value)
        self._publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._command_timeout_sec = float(self.get_parameter("command_timeout_sec").value)
        self._linear_x_max = abs(float(self.get_parameter("linear_x_max").value))
        self._linear_y_max = abs(float(self.get_parameter("linear_y_max").value))
        self._angular_z_max = abs(float(self.get_parameter("angular_z_max").value))

        self._client = self.create_client(RpcService, self._rpc_service_name)
        self._latest_cmd = Twist()
        self._latest_cmd_time = None
        self._last_sent: Optional[tuple[float, float, float]] = None
        self._pending_future = None
        self._service_wait_logged = False
        self._stale_stop_sent = True

        self.create_subscription(Twist, self._cmd_vel_topic, self._cmd_vel_callback, 10)
        self.create_timer(1.0 / max(self._publish_rate_hz, 1.0), self._tick)

        self.get_logger().info(
            "Bridging %s -> %s (kMove api_id=%d)"
            % (self._cmd_vel_topic, self._rpc_service_name, MOVE_API_ID)
        )

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self._latest_cmd = msg
        self._latest_cmd_time = self.get_clock().now()
        self._stale_stop_sent = False

    def _tick(self) -> None:
        if self._pending_future is not None:
            if not self._pending_future.done():
                return
            try:
                result = self._pending_future.result()
                if result is not None and result.msg.status != 0:
                    self.get_logger().warning(
                        "kMove returned status=%d body=%s"
                        % (result.msg.status, result.msg.body)
                    )
            except Exception as exc:  # pragma: no cover - transport/runtime dependent
                self.get_logger().warning(f"kMove RPC failed: {exc}")
            finally:
                self._pending_future = None

        if not self._client.wait_for_service(timeout_sec=0.0):
            if not self._service_wait_logged:
                self.get_logger().warning(
                    f"Waiting for locomotion RPC service {self._rpc_service_name}"
                )
                self._service_wait_logged = True
            return
        self._service_wait_logged = False

        target = self._desired_velocity()
        if target is None:
            return

        if self._last_sent is not None and all(
            math.isclose(a, b, abs_tol=EPSILON) for a, b in zip(target, self._last_sent)
        ):
            return

        self._send_move(*target)

    def _desired_velocity(self) -> Optional[tuple[float, float, float]]:
        if self._latest_cmd_time is None:
            return None

        age = (self.get_clock().now() - self._latest_cmd_time).nanoseconds / 1e9
        if age > self._command_timeout_sec:
            if self._stale_stop_sent:
                return None
            return (0.0, 0.0, 0.0)

        return (
            self._clip(self._latest_cmd.linear.x, self._linear_x_max),
            self._clip(self._latest_cmd.linear.y, self._linear_y_max),
            self._clip(self._latest_cmd.angular.z, self._angular_z_max),
        )

    def _send_move(self, vx: float, vy: float, vyaw: float) -> None:
        request = RpcService.Request()
        request.msg.api_id = MOVE_API_ID
        request.msg.body = json.dumps(
            {"vx": vx, "vy": vy, "vyaw": vyaw},
            separators=(",", ":"),
        )
        self._pending_future = self._client.call_async(request)
        self._last_sent = (vx, vy, vyaw)
        self._stale_stop_sent = vx == 0.0 and vy == 0.0 and vyaw == 0.0
        self.get_logger().info("Sent kMove vx=%.3f vy=%.3f vyaw=%.3f" % (vx, vy, vyaw))

    def stop_robot(self) -> None:
        if self._last_sent is None:
            return
        if all(math.isclose(value, 0.0, abs_tol=EPSILON) for value in self._last_sent):
            return
        if not self._client.wait_for_service(timeout_sec=1.0):
            return
        self._send_move(0.0, 0.0, 0.0)
        if self._pending_future is not None:
            rclpy.spin_until_future_complete(self, self._pending_future, timeout_sec=1.0)
            self._pending_future = None

    @staticmethod
    def _clip(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = K1CmdVelBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
