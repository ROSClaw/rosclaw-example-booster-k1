#!/usr/bin/env python3

import socket
import threading
from collections import defaultdict

import numpy as np
import rclpy
from booster_interface.msg import Odometer
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from .telemetry_stream_common import pack_odometer
from .telemetry_stream_common import pack_scan


ODOM_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

SCAN_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class K1TelemetryStreamServer(Node):
    def __init__(self) -> None:
        super().__init__("k1_telemetry_stream_server")

        self.declare_parameter("bind_host", "0.0.0.0")
        self.declare_parameter("port", 5601)
        self.declare_parameter("odometer_topic", "/odometer_state")
        self.declare_parameter("scan_topic", "/scan")

        self._bind_host = str(self.get_parameter("bind_host").value or "0.0.0.0")
        self._port = int(self.get_parameter("port").value)
        self._odometer_topic = str(self.get_parameter("odometer_topic").value)
        self._scan_topic = str(self.get_parameter("scan_topic").value)
        self._counts = defaultdict(int)

        self._client_lock = threading.Lock()
        self._client_socket: socket.socket | None = None
        self._server_socket: socket.socket | None = None
        self._stop_event = threading.Event()

        self.create_subscription(Odometer, self._odometer_topic, self._handle_odometer, ODOM_QOS)
        self.create_subscription(LaserScan, self._scan_topic, self._handle_scan, SCAN_QOS)
        self.create_timer(10.0, self._log_status)

        self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._accept_thread.start()

        self.get_logger().info(
            "Telemetry stream server %s:%d (%s, %s)"
            % (self._bind_host, self._port, self._odometer_topic, self._scan_topic)
        )

    def destroy_node(self) -> bool:
        self._stop_event.set()
        self._close_client_socket()
        if self._server_socket is not None:
            try:
                self._server_socket.close()
            except OSError:
                pass
            self._server_socket = None
        return super().destroy_node()

    def _accept_loop(self) -> None:
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((self._bind_host, self._port))
        server_socket.listen(1)
        server_socket.settimeout(1.0)
        self._server_socket = server_socket

        while not self._stop_event.is_set():
            try:
                client_socket, client_addr = server_socket.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            client_socket.settimeout(5.0)
            with self._client_lock:
                old_socket = self._client_socket
                self._client_socket = client_socket
            if old_socket is not None:
                try:
                    old_socket.close()
                except OSError:
                    pass
            self._counts["client_connect"] += 1
            self.get_logger().info("Telemetry client connected from %s:%s" % client_addr[:2])

    def _close_client_socket(self) -> None:
        with self._client_lock:
            client_socket = self._client_socket
            self._client_socket = None
        if client_socket is not None:
            try:
                client_socket.close()
            except OSError:
                pass

    def _send_packet(self, payload: bytes) -> bool:
        with self._client_lock:
            client_socket = self._client_socket
        if client_socket is None:
            return False
        try:
            client_socket.sendall(payload)
            return True
        except (BrokenPipeError, ConnectionError, OSError) as exc:
            self._counts["client_disconnect"] += 1
            self.get_logger().warning("Telemetry client disconnected: %s" % exc)
            self._close_client_socket()
            return False

    def _handle_odometer(self, msg: Odometer) -> None:
        stamp_ns = self.get_clock().now().nanoseconds
        self._counts["odometer_rx"] += 1
        if self._send_packet(pack_odometer(stamp_ns, float(msg.x), float(msg.y), float(msg.theta))):
            self._counts["odometer_pub"] += 1

    def _handle_scan(self, msg: LaserScan) -> None:
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if stamp_ns <= 0:
            stamp_ns = self.get_clock().now().nanoseconds
        self._counts["scan_rx"] += 1
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        packet = pack_scan(
            stamp_ns=stamp_ns,
            angle_min=float(msg.angle_min),
            angle_max=float(msg.angle_max),
            angle_increment=float(msg.angle_increment),
            time_increment=float(msg.time_increment),
            scan_time=float(msg.scan_time),
            range_min=float(msg.range_min),
            range_max=float(msg.range_max),
            ranges_payload=ranges.tobytes(),
            range_count=int(ranges.size),
        )
        if self._send_packet(packet):
            self._counts["scan_pub"] += 1

    def _log_status(self) -> None:
        self.get_logger().info(
            "telemetry counts connect=%d disconnect=%d odometer_rx=%d odometer_pub=%d scan_rx=%d scan_pub=%d"
            % (
                self._counts["client_connect"],
                self._counts["client_disconnect"],
                self._counts["odometer_rx"],
                self._counts["odometer_pub"],
                self._counts["scan_rx"],
                self._counts["scan_pub"],
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = K1TelemetryStreamServer()
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
