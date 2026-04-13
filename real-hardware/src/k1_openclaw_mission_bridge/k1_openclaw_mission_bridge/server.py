from __future__ import annotations

import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from rosclaw_autonomy_msgs.msg import AutonomyMode, RobotBelief

from .openclaw_client import OpenClawClient, OpenClawClientError


MODE_NAMES = {
    AutonomyMode.MANUAL: "MANUAL",
    AutonomyMode.ASSISTED: "ASSISTED",
    AutonomyMode.SUPERVISED_AUTONOMY: "SUPERVISED_AUTONOMY",
    AutonomyMode.FULL_AUTONOMY: "FULL_AUTONOMY",
    AutonomyMode.IDLE_ROAM: "IDLE_ROAM",
}


def yaw_from_quaternion(z: float, w: float) -> float:
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


class BridgeHTTPServer(ThreadingHTTPServer):
    def __init__(self, server_address: tuple[str, int], bridge: "K1MissionBridgeNode") -> None:
        self.bridge = bridge
        super().__init__(server_address, BridgeRequestHandler)


class BridgeRequestHandler(BaseHTTPRequestHandler):
    server: BridgeHTTPServer

    def do_GET(self) -> None:  # noqa: N802
        if self.path == "/health":
            self._write_json(200, {"ok": True, "ts": time.time()})
            return
        if self.path == "/api/state":
            self._write_json(200, self.server.bridge.build_state_snapshot())
            return
        self._write_json(404, {"ok": False, "summary": "not found"})

    def do_POST(self) -> None:  # noqa: N802
        try:
            content_length = int(self.headers.get("Content-Length", "0"))
            payload = self.rfile.read(content_length) if content_length > 0 else b"{}"
            body = json.loads(payload.decode("utf-8"))
        except json.JSONDecodeError:
            self._write_json(400, {"ok": False, "summary": "invalid JSON body"})
            return

        try:
            if self.path == "/api/map/alignment":
                response = self.server.bridge.handle_alignment(body)
            elif self.path == "/api/map/observations":
                response = self.server.bridge.handle_observation(body)
            elif self.path == "/api/goals/tap":
                response = self.server.bridge.handle_tap_goal(body)
            elif self.path == "/api/autonomy":
                response = self.server.bridge.handle_autonomy(body)
            elif self.path == "/api/report":
                response = self.server.bridge.handle_report()
            elif self.path == "/api/stop":
                response = self.server.bridge.handle_stop()
            else:
                self._write_json(404, {"ok": False, "summary": "not found"})
                return
        except (ValueError, OpenClawClientError) as exc:
            self._write_json(502, {"ok": False, "summary": str(exc)})
            return
        except Exception as exc:  # pragma: no cover - defensive server guard
            self._write_json(500, {"ok": False, "summary": str(exc)})
            return

        self._write_json(200, response)

    def log_message(self, _format: str, *_args) -> None:
        return

    def _write_json(self, status_code: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


class K1MissionBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("k1_openclaw_mission_bridge")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8088)
        self.declare_parameter("openclaw_binary", "openclaw")
        self.declare_parameter("openclaw_agent_id", "main")
        self.declare_parameter("openclaw_session_id", "k1-visionos")
        self.declare_parameter("openclaw_timeout_seconds", 45)
        self.declare_parameter("enable_openclaw", True)
        self.declare_parameter("spatial_observation_topic", "/visionos/spatial_observation")
        self.declare_parameter("alignment_topic", "/visionos/alignment")
        self.declare_parameter("mapping_status_topic", "/visionos/mapping_status")

        self._lock = threading.Lock()
        self._robot_pose: dict[str, Any] | None = None
        self._autonomy = {
            "enabled": False,
            "mode": "MANUAL",
            "nav_state": "idle",
            "last_outcome": "",
        }
        self._current_intent = "Waiting for robot belief"
        self._mapping = {
            "alignment_state": "collecting",
            "shared_map_status": "waiting_for_spatial_data",
            "observation_count": 0,
            "floor_point_count": 0,
        }
        self._last_goal: dict[str, Any] | None = None
        self._reports: list[dict[str, Any]] = []
        self._last_alignment: dict[str, Any] | None = None

        self._spatial_publisher = self.create_publisher(
            String,
            str(self.get_parameter("spatial_observation_topic").value),
            10,
        )
        self._alignment_publisher = self.create_publisher(
            String,
            str(self.get_parameter("alignment_topic").value),
            10,
        )
        self.create_subscription(RobotBelief, "/rosclaw/robot_belief", self._on_robot_belief, 10)
        self.create_subscription(AutonomyMode, "/rosclaw/autonomy_mode", self._on_autonomy_mode, 10)
        self.create_subscription(String, "/rosclaw/current_intent", self._on_current_intent, 10)
        self.create_subscription(
            String,
            str(self.get_parameter("mapping_status_topic").value),
            self._on_mapping_status,
            10,
        )

        self._openclaw = OpenClawClient(
            binary=str(self.get_parameter("openclaw_binary").value),
            agent_id=str(self.get_parameter("openclaw_agent_id").value),
            session_id=str(self.get_parameter("openclaw_session_id").value),
            timeout_seconds=int(self.get_parameter("openclaw_timeout_seconds").value),
            enabled=bool(self.get_parameter("enable_openclaw").value),
        )

        self._http_server = BridgeHTTPServer(
            (
                str(self.get_parameter("host").value),
                int(self.get_parameter("port").value),
            ),
            self,
        )
        self._http_thread = threading.Thread(target=self._http_server.serve_forever, daemon=True)
        self._http_thread.start()
        self.get_logger().info(
            "visionOS mission bridge listening on http://%s:%s"
            % (
                self.get_parameter("host").value,
                self.get_parameter("port").value,
            )
        )

    def destroy_node(self) -> bool:
        self._http_server.shutdown()
        self._http_server.server_close()
        return super().destroy_node()

    def _on_robot_belief(self, msg: RobotBelief) -> None:
        yaw = yaw_from_quaternion(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        with self._lock:
            self._robot_pose = {
                "frame": msg.pose.header.frame_id or "map",
                "position": {
                    "x": float(msg.pose.pose.position.x),
                    "y": float(msg.pose.pose.position.y),
                    "z": float(msg.pose.pose.position.z),
                },
                "yaw_radians": float(yaw),
            }
            self._autonomy["nav_state"] = msg.nav_state
            self._autonomy["last_outcome"] = msg.last_outcome

    def _on_autonomy_mode(self, msg: AutonomyMode) -> None:
        with self._lock:
            mode_name = MODE_NAMES.get(msg.mode, str(msg.mode))
            self._autonomy["mode"] = mode_name
            self._autonomy["enabled"] = mode_name in {"FULL_AUTONOMY", "IDLE_ROAM"}

    def _on_current_intent(self, msg: String) -> None:
        with self._lock:
            self._current_intent = msg.data or "Idle"

    def _on_mapping_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("mapping status was not valid JSON")
            return
        with self._lock:
            self._mapping.update(payload)

    def build_state_snapshot(self) -> dict[str, Any]:
        with self._lock:
            robot_pose = self._robot_pose
            autonomy = dict(self._autonomy)
            mapping = dict(self._mapping)
            last_goal = dict(self._last_goal) if self._last_goal else None
            reports = list(self._reports)
            current_intent = self._current_intent

        return {
            "ok": True,
            "connection_state": "connected" if robot_pose else "degraded",
            "robot_pose": robot_pose,
            "autonomy": autonomy,
            "mapping": mapping,
            "current_intent": current_intent,
            "status_summary": current_intent,
            "last_goal": last_goal,
            "reports": reports,
        }

    def handle_alignment(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._last_alignment = payload
        with self._lock:
            self._mapping["alignment_state"] = "aligned"
            self._mapping["shared_map_status"] = "aligned"
        self._publish_json(self._alignment_publisher, payload)
        return {"ok": True, "summary": "alignment updated"}

    def handle_observation(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._publish_json(self._spatial_publisher, payload)
        with self._lock:
            self._mapping["observation_count"] = max(
                int(self._mapping.get("observation_count", 0)),
                int(payload.get("sample_count", 0)),
            )
            self._mapping["floor_point_count"] = len(payload.get("floor_points", []))
            self._mapping["shared_map_status"] = "collecting" if payload.get("floor_points") else "tracking_headset"
        return {"ok": True, "summary": "observation accepted"}

    def handle_tap_goal(self, payload: dict[str, Any]) -> dict[str, Any]:
        map_point = payload.get("map_point") or {}
        frame = str(payload.get("frame", "map"))
        result = self._openclaw.navigate_to(
            float(map_point.get("x", 0.0)),
            float(map_point.get("y", 0.0)),
            float(map_point.get("z", 0.0)),
            frame,
        )
        with self._lock:
            self._last_goal = {
                "frame": frame,
                "position": map_point,
                "summary": result.get("summary", "navigation goal sent"),
            }
        return {
            "ok": bool(result.get("ok", False)),
            "summary": str(result.get("summary", "navigation request completed")),
        }

    def handle_autonomy(self, payload: dict[str, Any]) -> dict[str, Any]:
        result = self._openclaw.set_autonomy(bool(payload.get("enabled", False)))
        return {
            "ok": bool(result.get("ok", False)),
            "summary": str(result.get("summary", "autonomy request completed")),
        }

    def handle_report(self) -> dict[str, Any]:
        result = self._openclaw.request_report()
        report = result.get("report") or {}
        summary = str(result.get("summary", "scene report completed"))
        timestamp = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        with self._lock:
            self._reports.insert(
                0,
                {
                    "id": f"report-{int(time.time() * 1000)}",
                    "timestamp": timestamp,
                    "summary": str(report.get("summary", summary)),
                    "labels": list(report.get("labels", [])),
                },
            )
            self._reports = self._reports[:10]
        return {"ok": bool(result.get("ok", False)), "summary": summary}

    def handle_stop(self) -> dict[str, Any]:
        result = self._openclaw.stop()
        return {
            "ok": bool(result.get("ok", False)),
            "summary": str(result.get("summary", "stop request completed")),
        }

    def _publish_json(self, publisher, payload: dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = K1MissionBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
