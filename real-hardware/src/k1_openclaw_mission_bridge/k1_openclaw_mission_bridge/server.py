from __future__ import annotations

import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable

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
        try:
            self.send_response(status_code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            return


class K1MissionBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("k1_openclaw_mission_bridge")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8088)
        self.declare_parameter("openclaw_binary", "openclaw")
        self.declare_parameter("openclaw_agent_id", "main")
        self.declare_parameter("openclaw_session_id", "k1-visionos")
        self.declare_parameter("openclaw_timeout_seconds", 10)
        self.declare_parameter("enable_openclaw", True)
        self.declare_parameter("spatial_observation_topic", "/visionos/spatial_observation")
        self.declare_parameter("alignment_topic", "/visionos/alignment")
        self.declare_parameter("mapping_status_topic", "/visionos/mapping_status")

        self._lock = threading.Lock()
        self._openclaw_lock = threading.Lock()
        self._openclaw_active_request: str | None = None
        self._openclaw_active_since: float | None = None
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
        self._agent_activity: list[dict[str, Any]] = []
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
            agent_activity = list(self._agent_activity)
            current_intent = self._current_intent
            openclaw_active_request = self._openclaw_active_request
            openclaw_active_since = self._openclaw_active_since

        status_summary = current_intent
        if openclaw_active_request:
            elapsed_seconds = 0
            if openclaw_active_since is not None:
                elapsed_seconds = max(0, int(time.time() - openclaw_active_since))
            status_summary = (
                f"OpenClaw is processing {openclaw_active_request} "
                f"({elapsed_seconds}s elapsed)."
            )

        return {
            "ok": True,
            "connection_state": "connected" if robot_pose else "degraded",
            "robot_pose": robot_pose,
            "autonomy": autonomy,
            "mapping": mapping,
            "current_intent": current_intent,
            "status_summary": status_summary,
            "last_goal": last_goal,
            "reports": reports,
            "agent_activity": agent_activity,
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
        goal_position = {
            "x": float(map_point.get("x", 0.0)),
            "y": float(map_point.get("y", 0.0)),
            "z": float(map_point.get("z", 0.0)),
        }
        self._append_agent_activity(
            role="operator",
            summary="Requested tap-to-move.",
            detail=(
                f"Target floor point: ({goal_position['x']:.3f}, "
                f"{goal_position['y']:.3f}, "
                f"{goal_position['z']:.3f}) in {frame}."
            ),
            status="pending",
        )
        summary = self._submit_openclaw_request(
            "tap goal",
            lambda: self._openclaw.navigate_to(
                goal_position["x"],
                goal_position["y"],
                goal_position["z"],
                frame,
            ),
            accepted_summary="Tap goal queued. OpenClaw is deciding how to reach the floor target with Nav2.",
            on_success=lambda result: self._store_successful_goal(result, frame, goal_position),
        )
        return {
            "ok": True,
            "summary": summary,
        }

    def handle_autonomy(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._append_agent_activity(
            role="operator",
            summary="Requested autonomy change.",
            detail="Enable roaming." if bool(payload.get("enabled", False)) else "Return to MANUAL mode.",
            status="pending",
        )
        summary = self._submit_openclaw_request(
            "autonomy toggle",
            lambda: self._openclaw.set_autonomy(bool(payload.get("enabled", False))),
            accepted_summary=(
                "Autonomy change queued. OpenClaw is updating the robot mode."
            ),
        )
        return {
            "ok": True,
            "summary": summary,
        }

    def handle_report(self) -> dict[str, Any]:
        self._append_agent_activity(
            role="operator",
            summary="Requested scene report.",
            detail="Summarize what the robot currently sees.",
            status="pending",
        )
        summary = self._submit_openclaw_request(
            "scene report",
            self._openclaw.request_report,
            accepted_summary="Scene report queued. OpenClaw is gathering the current robot view.",
            on_success=self._store_successful_report,
        )
        return {"ok": True, "summary": summary}

    def handle_stop(self) -> dict[str, Any]:
        self._append_agent_activity(
            role="operator",
            summary="Requested stop.",
            detail="Emergency stop or cancel the current robot behavior.",
            status="pending",
        )
        summary = self._submit_openclaw_request(
            "stop",
            self._openclaw.stop,
            accepted_summary="Stop request queued. OpenClaw is taking the safest available stop path.",
            on_success=lambda _result: self._clear_last_goal(),
        )
        return {
            "ok": True,
            "summary": summary,
        }

    def _publish_json(self, publisher, payload: dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        publisher.publish(msg)

    def _append_agent_activity(
        self,
        role: str,
        summary: str,
        detail: str,
        status: str,
    ) -> None:
        entry = {
            "id": f"activity-{time.time_ns()}",
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "role": role,
            "summary": summary,
            "detail": detail,
            "status": status,
        }
        with self._lock:
            self._agent_activity.insert(0, entry)
            self._agent_activity = self._agent_activity[:20]

    def _record_openclaw_activity(self, result: dict[str, Any]) -> None:
        status = "success" if bool(result.get("ok", False)) else "error"

        decision = result.get("decision")
        if isinstance(decision, dict):
            detail_parts = []
            for key in ("skill", "tool", "action", "action_type", "service", "target_mode", "intent", "frame"):
                value = decision.get(key)
                if value:
                    detail_parts.append(f"{key}={value}")
            self._append_agent_activity(
                role="assistant",
                summary=str(result.get("summary", "OpenClaw handled the request.")),
                detail=", ".join(detail_parts) if detail_parts else "Operator-visible decision trace returned by OpenClaw.",
                status=status,
            )
        else:
            self._append_agent_activity(
                role="assistant",
                summary=str(result.get("summary", "OpenClaw handled the request.")),
                detail="Operator-visible decision trace was not provided; showing the command summary only.",
                status=status,
            )

        activity = result.get("activity")
        if isinstance(activity, list):
            for item in reversed(activity[:4]):
                if not isinstance(item, dict):
                    continue
                self._append_agent_activity(
                    role=str(item.get("role", "assistant")),
                    summary=str(item.get("summary", result.get("summary", "OpenClaw activity"))),
                    detail=str(item.get("detail", "")),
                    status=status,
                )

    def _submit_openclaw_request(
        self,
        label: str,
        action: Callable[[], dict[str, Any]],
        accepted_summary: str,
        on_success: Callable[[dict[str, Any]], None] | None = None,
    ) -> str:
        with self._lock:
            active_request = self._openclaw_active_request
            if active_request is not None:
                raise OpenClawClientError(
                    f"OpenClaw is still processing {active_request}. Wait for it to finish before sending another command."
                )
            self._openclaw_active_request = label
            self._openclaw_active_since = time.time()

        self._append_agent_activity(
            role="assistant",
            summary=f"OpenClaw is processing {label}.",
            detail="The request was accepted by the bridge and is being handled in the background.",
            status="pending",
        )

        worker = threading.Thread(
            target=self._execute_openclaw_request,
            args=(label, action, on_success),
            daemon=True,
        )
        worker.start()
        return accepted_summary

    def _execute_openclaw_request(
        self,
        label: str,
        action: Callable[[], dict[str, Any]],
        on_success: Callable[[dict[str, Any]], None] | None,
    ) -> None:
        try:
            with self._openclaw_lock:
                result = action()
            self._record_openclaw_activity(result)
            if bool(result.get("ok", False)) and on_success is not None:
                on_success(result)
        except OpenClawClientError as exc:
            self._append_agent_activity(
                role="assistant",
                summary=f"OpenClaw {label} failed.",
                detail=str(exc),
                status="error",
            )
        except Exception as exc:  # pragma: no cover - defensive worker guard
            self.get_logger().exception("unexpected error while processing %s", label)
            self._append_agent_activity(
                role="assistant",
                summary=f"OpenClaw {label} crashed.",
                detail=str(exc),
                status="error",
            )
        finally:
            with self._lock:
                self._openclaw_active_request = None
                self._openclaw_active_since = None

    def _store_successful_goal(
        self,
        result: dict[str, Any],
        frame: str,
        goal_position: dict[str, float],
    ) -> None:
        with self._lock:
            self._last_goal = {
                "frame": frame,
                "position": goal_position,
                "summary": result.get("summary", "navigation goal sent"),
            }

    def _store_successful_report(self, result: dict[str, Any]) -> None:
        report = result.get("report") or {}
        if not report:
            return

        timestamp = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        summary = str(result.get("summary", "scene report completed"))
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

    def _clear_last_goal(self) -> None:
        with self._lock:
            self._last_goal = None


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
