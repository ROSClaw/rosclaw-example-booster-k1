#!/usr/bin/env python3

import socket
import threading
import time
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

from .telemetry_stream_common import ODOM_PACKET
from .telemetry_stream_common import ODOM_PAYLOAD
from .telemetry_stream_common import SCAN_META
from .telemetry_stream_common import SCAN_PACKET
from .telemetry_stream_common import recv_packet


ODOM_PUB_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

SCAN_PUB_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class K1TelemetryStreamClient(Node):
    def __init__(self) -> None:
        super().__init__("k1_telemetry_stream_client")

        self.declare_parameter("server_host", "192.168.2.152")
        self.declare_parameter("server_port", 5601)
        self.declare_parameter("odometer_output_topic", "/k1/odometer_state")
        self.declare_parameter("scan_output_topic", "/k1/scan")
        self.declare_parameter("scan_frame_id", "trunk_link")
        self.declare_parameter("reconnect_sec", 2.0)

        self._server_host = str(self.get_parameter("server_host").value)
        self._server_port = int(self.get_parameter("server_port").value)
        self._odometer_output_topic = str(self.get_parameter("odometer_output_topic").value)
        self._scan_output_topic = str(self.get_parameter("scan_output_topic").value)
        self._scan_frame_id = str(self.get_parameter("scan_frame_id").value)
        self._reconnect_sec = float(self.get_parameter("reconnect_sec").value)
        self._counts = defaultdict(int)

        self._odometer_pub = self.create_publisher(Odometer, self._odometer_output_topic, ODOM_PUB_QOS)
        self._scan_pub = self.create_publisher(LaserScan, self._scan_output_topic, SCAN_PUB_QOS)
        self.create_timer(10.0, self._log_status)

        self._stop_event = threading.Event()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        self.get_logger().info(
            "Telemetry stream client %s:%d -> (%s, %s)"
            % (self._server_host, self._server_port, self._odometer_output_topic, self._scan_output_topic)
        )

    def destroy_node(self) -> bool:
        self._stop_event.set()
        return super().destroy_node()

    def _reader_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                with socket.create_connection((self._server_host, self._server_port), timeout=5.0) as sock:
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    sock.settimeout(5.0)
                    self._counts["connect"] += 1
                    self.get_logger().info(
                        "Connected to telemetry stream server %s:%d"
                        % (self._server_host, self._server_port)
                    )
                    while not self._stop_event.is_set():
                        kind, stamp_ns, payload = recv_packet(sock)
                        if kind == ODOM_PACKET:
                            self._handle_odometer(payload)
                        elif kind == SCAN_PACKET:
                            self._handle_scan(stamp_ns, payload)
                        else:
                            self.get_logger().warning("Ignoring unknown telemetry packet kind %d" % kind)
            except (ConnectionError, OSError, ValueError) as exc:
                self._counts["disconnect"] += 1
                if not self._stop_event.is_set():
                    self.get_logger().warning("Telemetry stream connection issue: %s" % exc)
                    time.sleep(self._reconnect_sec)

    def _handle_odometer(self, payload: bytes) -> None:
        x, y, theta = ODOM_PAYLOAD.unpack(payload)
        msg = Odometer()
        msg.x = float(x)
        msg.y = float(y)
        msg.theta = float(theta)
        self._odometer_pub.publish(msg)
        self._counts["odometer_rx"] += 1
        self._counts["odometer_pub"] += 1

    def _handle_scan(self, stamp_ns: int, payload: bytes) -> None:
        meta = payload[: SCAN_META.size]
        data = payload[SCAN_META.size :]
        (
            angle_min,
            angle_max,
            angle_increment,
            time_increment,
            scan_time,
            range_min,
            range_max,
            range_count,
        ) = SCAN_META.unpack(meta)
        ranges = np.frombuffer(data, dtype=np.float32, count=range_count)

        msg = LaserScan()
        msg.header.stamp.sec = stamp_ns // 1_000_000_000
        msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
        msg.header.frame_id = self._scan_frame_id
        msg.angle_min = float(angle_min)
        msg.angle_max = float(angle_max)
        msg.angle_increment = float(angle_increment)
        msg.time_increment = float(time_increment)
        msg.scan_time = float(scan_time)
        msg.range_min = float(range_min)
        msg.range_max = float(range_max)
        msg.ranges = ranges.tolist()
        msg.intensities = []
        self._scan_pub.publish(msg)
        self._counts["scan_rx"] += 1
        self._counts["scan_pub"] += 1

    def _log_status(self) -> None:
        self.get_logger().info(
            "telemetry client counts connect=%d disconnect=%d odometer_rx=%d odometer_pub=%d scan_rx=%d scan_pub=%d"
            % (
                self._counts["connect"],
                self._counts["disconnect"],
                self._counts["odometer_rx"],
                self._counts["odometer_pub"],
                self._counts["scan_rx"],
                self._counts["scan_pub"],
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = K1TelemetryStreamClient()
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
