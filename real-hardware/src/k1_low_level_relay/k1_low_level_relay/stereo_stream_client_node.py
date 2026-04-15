#!/usr/bin/env python3

import queue
import socket
import threading
import time
from collections import defaultdict

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image

from .stereo_stream_common import LEFT_EYE
from .stereo_stream_common import RIGHT_EYE
from .stereo_stream_common import recv_frame

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


IMAGE_PUB_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class K1StereoStreamClient(Node):
    def __init__(self) -> None:
        super().__init__("k1_stereo_stream_client")

        if cv2 is None:
            raise RuntimeError("OpenCV is required for stereo streaming")

        self.declare_parameter("server_host", "192.168.2.152")
        self.declare_parameter("server_port", 5600)
        self.declare_parameter("left_output_topic", "/k1/image_left_raw")
        self.declare_parameter("right_output_topic", "/k1/image_right_raw")
        self.declare_parameter("left_frame_id", "k1_left_camera")
        self.declare_parameter("right_frame_id", "k1_right_camera")
        self.declare_parameter("reconnect_sec", 2.0)

        self._server_host = self.get_parameter("server_host").get_parameter_value().string_value
        self._server_port = int(self.get_parameter("server_port").get_parameter_value().integer_value)
        self._left_output_topic = self.get_parameter("left_output_topic").get_parameter_value().string_value
        self._right_output_topic = self.get_parameter("right_output_topic").get_parameter_value().string_value
        self._left_frame_id = self.get_parameter("left_frame_id").get_parameter_value().string_value
        self._right_frame_id = self.get_parameter("right_frame_id").get_parameter_value().string_value
        self._reconnect_sec = float(self.get_parameter("reconnect_sec").value)

        self._counts = defaultdict(int)
        self._left_pub = self.create_publisher(Image, self._left_output_topic, IMAGE_PUB_QOS)
        self._right_pub = self.create_publisher(Image, self._right_output_topic, IMAGE_PUB_QOS)
        self._frame_queue: queue.Queue[tuple[int, int, np.ndarray]] = queue.Queue(maxsize=16)
        self._stop_event = threading.Event()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()
        self._publish_timer = self.create_timer(0.01, self._publish_pending_frames)
        self._log_timer = self.create_timer(10.0, self._log_status)

        self.get_logger().info(
            "Stereo stream client %s:%d -> (%s, %s)"
            % (self._server_host, self._server_port, self._left_output_topic, self._right_output_topic)
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
                    self.get_logger().info("Connected to stereo stream server %s:%d" % (self._server_host, self._server_port))
                    while not self._stop_event.is_set():
                        eye_id, stamp_ns, payload = recv_frame(sock)
                        image = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
                        if image is None:
                            self._counts["decode_error"] += 1
                            continue
                        self._enqueue_frame((eye_id, stamp_ns, image))
                        if eye_id == LEFT_EYE:
                            self._counts["left_rx"] += 1
                        else:
                            self._counts["right_rx"] += 1
            except (ConnectionError, OSError, ValueError) as exc:
                self._counts["disconnect"] += 1
                if not self._stop_event.is_set():
                    self.get_logger().warning("Stereo stream connection issue: %s" % exc)
                    time.sleep(self._reconnect_sec)

    def _enqueue_frame(self, item: tuple[int, int, np.ndarray]) -> None:
        try:
            self._frame_queue.put_nowait(item)
        except queue.Full:
            try:
                self._frame_queue.get_nowait()
            except queue.Empty:
                pass
            self._frame_queue.put_nowait(item)
            self._counts["queue_drop"] += 1

    def _publish_pending_frames(self) -> None:
        while True:
            try:
                eye_id, stamp_ns, image = self._frame_queue.get_nowait()
            except queue.Empty:
                break
            msg = self._to_image_message(eye_id, stamp_ns, image)
            if eye_id == LEFT_EYE:
                self._left_pub.publish(msg)
                self._counts["left_pub"] += 1
            else:
                self._right_pub.publish(msg)
                self._counts["right_pub"] += 1

    def _to_image_message(self, eye_id: int, stamp_ns: int, image: np.ndarray) -> Image:
        msg = Image()
        if stamp_ns > 0:
            msg.header.stamp.sec = stamp_ns // 1_000_000_000
            msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._left_frame_id if eye_id == LEFT_EYE else self._right_frame_id

        if image.ndim == 2:
            msg.height = int(image.shape[0])
            msg.width = int(image.shape[1])
            msg.encoding = "mono8"
            msg.is_bigendian = 0
            msg.step = int(image.shape[1])
            msg.data = image.tobytes()
            return msg

        msg.height = int(image.shape[0])
        msg.width = int(image.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = int(image.shape[1] * image.shape[2])
        msg.data = image.tobytes()
        return msg

    def _log_status(self) -> None:
        self.get_logger().info(
            "client counts connect=%d disconnect=%d queue_drop=%d decode_error=%d left_rx=%d left_pub=%d right_rx=%d right_pub=%d"
            % (
                self._counts["connect"],
                self._counts["disconnect"],
                self._counts["queue_drop"],
                self._counts["decode_error"],
                self._counts["left_rx"],
                self._counts["left_pub"],
                self._counts["right_rx"],
                self._counts["right_pub"],
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = K1StereoStreamClient()
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
