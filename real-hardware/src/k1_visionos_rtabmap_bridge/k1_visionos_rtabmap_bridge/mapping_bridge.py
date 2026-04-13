from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class VisionOSMappingBridge(Node):
    def __init__(self) -> None:
        super().__init__("k1_visionos_rtabmap_bridge")

        self.declare_parameter("observation_topic", "/visionos/spatial_observation")
        self.declare_parameter("alignment_topic", "/visionos/alignment")
        self.declare_parameter("status_topic", "/visionos/mapping_status")
        self.declare_parameter("log_path", "/tmp/k1_visionos_mapping.ndjson")

        self._observation_count = 0
        self._floor_point_count = 0
        self._last_observation_ts: float | None = None
        self._alignment_state = "collecting"
        self._shared_map_status = "waiting_for_spatial_data"
        self._latest_device_position: dict[str, Any] | None = None

        self._status_publisher = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("observation_topic").value),
            self._on_observation,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("alignment_topic").value),
            self._on_alignment,
            10,
        )
        self.create_timer(1.0, self._publish_status)

        self._log_path = Path(str(self.get_parameter("log_path").value))
        self._log_path.parent.mkdir(parents=True, exist_ok=True)

    def _on_observation(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("spatial observation payload was not valid JSON")
            return

        self._observation_count = max(self._observation_count, int(payload.get("sample_count", 0)))
        self._floor_point_count = int(len(payload.get("floor_points", [])))
        self._last_observation_ts = time.time()
        self._latest_device_position = payload.get("device_position")
        self._shared_map_status = "collecting" if self._floor_point_count else "tracking_headset"
        self._append_log("observation", payload)

    def _on_alignment(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("alignment payload was not valid JSON")
            return
        self._alignment_state = "aligned"
        self._shared_map_status = "aligned"
        self._append_log("alignment", payload)

    def _publish_status(self) -> None:
        age_sec = None if self._last_observation_ts is None else max(0.0, time.time() - self._last_observation_ts)
        payload = {
            "alignment_state": self._alignment_state,
            "shared_map_status": self._shared_map_status,
            "observation_count": self._observation_count,
            "floor_point_count": self._floor_point_count,
            "last_observation_age_sec": age_sec,
            "latest_device_position": self._latest_device_position,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._status_publisher.publish(msg)

    def _append_log(self, kind: str, payload: dict[str, Any]) -> None:
        record = {
            "kind": kind,
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "payload": payload,
        }
        with self._log_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record) + "\n")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionOSMappingBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
