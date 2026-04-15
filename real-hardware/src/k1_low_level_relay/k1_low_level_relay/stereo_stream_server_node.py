#!/usr/bin/env python3

import socket
import threading
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
from .stereo_stream_common import pack_frame

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


IMAGE_SUB_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class K1StereoStreamServer(Node):
    def __init__(self) -> None:
        super().__init__("k1_stereo_stream_server")

        if cv2 is None:
            raise RuntimeError("OpenCV is required for stereo streaming")

        self.declare_parameter("bind_host", "0.0.0.0")
        self.declare_parameter("port", 5600)
        self.declare_parameter("left_input_topic", "/image_left_raw")
        self.declare_parameter("right_input_topic", "/image_right_raw")
        self.declare_parameter("output_encoding", "mono8")
        self.declare_parameter("output_width", 512)
        self.declare_parameter("output_height", 512)
        self.declare_parameter("resize_mode", "crop")
        self.declare_parameter("max_fps", 12.0)
        self.declare_parameter("jpeg_quality", 80)

        self._bind_host = self.get_parameter("bind_host").get_parameter_value().string_value or "0.0.0.0"
        self._port = int(self.get_parameter("port").get_parameter_value().integer_value)
        self._left_input_topic = self.get_parameter("left_input_topic").get_parameter_value().string_value
        self._right_input_topic = self.get_parameter("right_input_topic").get_parameter_value().string_value
        self._output_encoding = self._resolve_output_encoding()
        self._output_width = int(self.get_parameter("output_width").get_parameter_value().integer_value)
        self._output_height = int(self.get_parameter("output_height").get_parameter_value().integer_value)
        self._resize_mode = self._resolve_resize_mode()
        self._min_period_ns = self._resolve_min_period_ns()
        self._jpeg_quality = self._resolve_jpeg_quality()

        self._counts = defaultdict(int)
        self._latest_frames = {LEFT_EYE: None, RIGHT_EYE: None}
        self._latest_versions = {LEFT_EYE: 0, RIGHT_EYE: 0}
        self._sent_versions = {LEFT_EYE: 0, RIGHT_EYE: 0}
        self._latest_lock = threading.Lock()
        self._client_lock = threading.Lock()
        self._client_socket: socket.socket | None = None
        self._server_socket: socket.socket | None = None
        self._stop_event = threading.Event()

        self._left_subscription = self.create_subscription(
            Image, self._left_input_topic, self._on_left_image, IMAGE_SUB_QOS
        )
        self._right_subscription = self.create_subscription(
            Image, self._right_input_topic, self._on_right_image, IMAGE_SUB_QOS
        )
        self._send_timer = self.create_timer(1.0 / max(self.get_parameter("max_fps").value, 1.0), self._send_latest_frames)
        self._log_timer = self.create_timer(10.0, self._log_status)

        self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._accept_thread.start()

        self.get_logger().info(
            "Stereo stream server %s:%d (%s -> %s, %s -> %s, encoding=%s, resize=%dx%d, mode=%s, max_fps=%.2f, jpeg_quality=%d)"
            % (
                self._bind_host,
                self._port,
                self._left_input_topic,
                "left",
                self._right_input_topic,
                "right",
                self._output_encoding,
                self._output_width,
                self._output_height,
                self._resize_mode,
                self.get_parameter("max_fps").value,
                self._jpeg_quality,
            )
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

    def _resolve_output_encoding(self) -> str:
        configured = self.get_parameter("output_encoding").get_parameter_value().string_value.strip().lower()
        if configured in ("mono8", "rgb8"):
            return configured
        self.get_logger().warning("Unsupported output_encoding '%s', using mono8" % configured)
        return "mono8"

    def _resolve_resize_mode(self) -> str:
        configured = self.get_parameter("resize_mode").get_parameter_value().string_value.strip().lower()
        if configured in ("crop", "stretch"):
            return configured
        self.get_logger().warning("Unsupported resize_mode '%s', using crop" % configured)
        return "crop"

    def _resolve_min_period_ns(self) -> int:
        max_fps = float(self.get_parameter("max_fps").value)
        if max_fps <= 0.0:
            return 0
        return int(1e9 / max_fps)

    def _resolve_jpeg_quality(self) -> int:
        quality = int(self.get_parameter("jpeg_quality").value)
        return max(30, min(95, quality))

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
            self.get_logger().info("Stereo stream client connected from %s:%s" % client_addr[:2])

    def _close_client_socket(self) -> None:
        with self._client_lock:
            client_socket = self._client_socket
            self._client_socket = None
        if client_socket is not None:
            try:
                client_socket.close()
            except OSError:
                pass

    def _on_left_image(self, msg: Image) -> None:
        self._store_latest(LEFT_EYE, msg)
        self._counts["left_rx"] += 1

    def _on_right_image(self, msg: Image) -> None:
        self._store_latest(RIGHT_EYE, msg)
        self._counts["right_rx"] += 1

    def _store_latest(self, eye_id: int, msg: Image) -> None:
        with self._latest_lock:
            self._latest_versions[eye_id] += 1
            self._latest_frames[eye_id] = msg

    def _send_latest_frames(self) -> None:
        with self._client_lock:
            client_socket = self._client_socket
        if client_socket is None:
            return

        with self._latest_lock:
            work_items = []
            for eye_id in (LEFT_EYE, RIGHT_EYE):
                version = self._latest_versions[eye_id]
                if version > self._sent_versions[eye_id] and self._latest_frames[eye_id] is not None:
                    work_items.append((eye_id, version, self._latest_frames[eye_id]))

        for eye_id, version, msg in work_items:
            payload = self._encode_message(msg)
            if payload is None:
                key = "left_convert_error" if eye_id == LEFT_EYE else "right_convert_error"
                self._counts[key] += 1
                continue
            stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
            try:
                client_socket.sendall(pack_frame(eye_id, stamp_ns, payload))
            except (BrokenPipeError, ConnectionError, OSError) as exc:
                self._counts["client_disconnect"] += 1
                self.get_logger().warning("Stereo stream client disconnected: %s" % exc)
                self._close_client_socket()
                return

            self._sent_versions[eye_id] = version
            if eye_id == LEFT_EYE:
                self._counts["left_pub"] += 1
            else:
                self._counts["right_pub"] += 1

    def _encode_message(self, msg: Image) -> bytes | None:
        converted = self._convert_image_message(msg)
        if converted is None:
            return None
        success, encoded = cv2.imencode(
            ".jpg",
            converted,
            [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality],
        )
        if not success:
            self.get_logger().warning("Failed to JPEG-encode stereo frame")
            return None
        return encoded.tobytes()

    def _convert_image_message(self, msg: Image) -> np.ndarray | None:
        input_encoding = msg.encoding.strip().lower()
        if input_encoding == "nv12":
            return self._convert_nv12(msg)
        if input_encoding in ("mono8", "8uc1"):
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((int(msg.height), int(msg.width)))
            return self._resize_image(frame)
        if input_encoding in ("rgb8", "bgr8"):
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((int(msg.height), int(msg.width), 3))
            if input_encoding == "rgb8" and self._output_encoding == "rgb8":
                bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                return self._resize_image(bgr)
            if input_encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            elif self._output_encoding == "mono8":
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            return self._resize_image(frame)
        self.get_logger().warning("Unsupported stereo input encoding %s" % (input_encoding or "<empty>"))
        return None

    def _convert_nv12(self, msg: Image) -> np.ndarray | None:
        width = int(msg.width)
        height = int(msg.height)
        expected_len = width * height * 3 // 2
        if len(msg.data) != expected_len:
            self.get_logger().warning(
                "Skipping NV12 frame with unexpected size %d (expected %d)" % (len(msg.data), expected_len)
            )
            return None

        frame = np.frombuffer(msg.data, dtype=np.uint8)
        y_plane = frame[: width * height].reshape((height, width))
        if self._output_encoding == "mono8":
            return self._resize_image(y_plane)

        nv12 = frame.reshape((height * 3 // 2, width))
        bgr = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
        return self._resize_image(bgr)

    def _resize_image(self, image: np.ndarray) -> np.ndarray:
        target_width = self._output_width
        target_height = self._output_height
        source_height, source_width = image.shape[:2]

        if target_width <= 0 and target_height <= 0:
            return image

        if target_width > 0 and target_height > 0:
            if self._resize_mode == "stretch":
                interpolation = cv2.INTER_AREA if target_width <= source_width and target_height <= source_height else cv2.INTER_LINEAR
                return cv2.resize(image, (target_width, target_height), interpolation=interpolation)

            scale = max(target_width / float(source_width), target_height / float(source_height))
            new_width = max(1, int(round(source_width * scale)))
            new_height = max(1, int(round(source_height * scale)))
            interpolation = cv2.INTER_AREA if scale <= 1.0 else cv2.INTER_LINEAR
            resized = cv2.resize(image, (new_width, new_height), interpolation=interpolation)
            x0 = max(0, (new_width - target_width) // 2)
            y0 = max(0, (new_height - target_height) // 2)
            return resized[y0 : y0 + target_height, x0 : x0 + target_width]

        if target_width > 0:
            scale = target_width / float(source_width)
            new_width = target_width
            new_height = max(1, int(round(source_height * scale)))
        else:
            scale = target_height / float(source_height)
            new_height = target_height
            new_width = max(1, int(round(source_width * scale)))

        if new_width == source_width and new_height == source_height:
            return image

        interpolation = cv2.INTER_AREA if new_width <= source_width and new_height <= source_height else cv2.INTER_LINEAR
        return cv2.resize(image, (new_width, new_height), interpolation=interpolation)

    def _log_status(self) -> None:
        self.get_logger().info(
            "stream counts client_connect=%d client_disconnect=%d left_rx=%d left_pub=%d left_convert_error=%d right_rx=%d right_pub=%d right_convert_error=%d"
            % (
                self._counts["client_connect"],
                self._counts["client_disconnect"],
                self._counts["left_rx"],
                self._counts["left_pub"],
                self._counts["left_convert_error"],
                self._counts["right_rx"],
                self._counts["right_pub"],
                self._counts["right_convert_error"],
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = K1StereoStreamServer()
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
