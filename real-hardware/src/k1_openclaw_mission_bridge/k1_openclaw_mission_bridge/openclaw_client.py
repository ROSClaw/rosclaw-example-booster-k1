from __future__ import annotations

import json
import subprocess
from dataclasses import dataclass
from typing import Any


class OpenClawClientError(RuntimeError):
    """Raised when the OpenClaw CLI fails or returns malformed payloads."""


@dataclass(slots=True)
class OpenClawClient:
    binary: str
    agent_id: str
    session_id: str
    timeout_seconds: int = 45
    enabled: bool = True

    def navigate_to(self, x: float, y: float, z: float, frame: str) -> dict[str, Any]:
        prompt = f"""
You are controlling a Booster K1 mobile robot through OpenClaw with ROS access.

Send exactly one navigation goal in the `{frame}` frame to:
- x: {x:.3f}
- y: {y:.3f}
- z: {z:.3f}

Rules:
- Prefer the existing ROSClaw navigation skill or `ros2_action_goal`.
- Do not use raw velocity control for this request.
- If navigation is unavailable, fail instead of improvising.

Return ONLY JSON:
{{"ok":true,"summary":"navigation goal sent","goal":{{"frame":"{frame}","x":{x:.3f},"y":{y:.3f},"z":{z:.3f}}}}}

Failure JSON:
{{"ok":false,"summary":"navigation goal failed","goal":null}}
""".strip()
        return self._run_prompt(prompt)

    def set_autonomy(self, enabled: bool) -> dict[str, Any]:
        if enabled:
            prompt = """
You are controlling a Booster K1 mobile robot through OpenClaw with ROS access.

Enable autonomous roaming for the robot.

Rules:
- Use `/rosclaw/set_autonomy_mode`.
- Set the mode to FULL_AUTONOMY so the autonomy node can roam on its own.
- Do not use manual velocity streaming.

Return ONLY JSON:
{"ok":true,"summary":"autonomy enabled"}

Failure JSON:
{"ok":false,"summary":"autonomy enable failed"}
""".strip()
        else:
            prompt = """
You are controlling a Booster K1 mobile robot through OpenClaw with ROS access.

Disable autonomous roaming and return the robot to MANUAL mode.

Rules:
- Use `/rosclaw/set_autonomy_mode`.
- Set the mode to MANUAL.

Return ONLY JSON:
{"ok":true,"summary":"autonomy disabled"}

Failure JSON:
{"ok":false,"summary":"autonomy disable failed"}
""".strip()
        return self._run_prompt(prompt)

    def request_report(self) -> dict[str, Any]:
        prompt = """
You are controlling a Booster K1 mobile robot through OpenClaw with ROS access.

Capture what the robot currently sees and summarize it for a visionOS operator.

Rules:
- Use the existing ROSClaw perception or camera tools.
- Return a short summary and a small label list.
- If a snapshot cannot be taken, still summarize the latest scene state if available.

Return ONLY JSON:
{"ok":true,"summary":"scene report ready","report":{"summary":"short scene summary","labels":["label1","label2"]}}

Failure JSON:
{"ok":false,"summary":"scene report failed","report":null}
""".strip()
        return self._run_prompt(prompt)

    def stop(self) -> dict[str, Any]:
        prompt = """
You are controlling a Booster K1 mobile robot through OpenClaw with ROS access.

Issue an immediate software stop.

Rules:
- Prefer the ROSClaw emergency stop path if available.
- If that is unavailable, cancel the active navigation/autonomy behavior and stop the robot.

Return ONLY JSON:
{"ok":true,"summary":"stop sent"}

Failure JSON:
{"ok":false,"summary":"stop failed"}
""".strip()
        return self._run_prompt(prompt)

    def _run_prompt(self, prompt: str) -> dict[str, Any]:
        if not self.enabled:
            raise OpenClawClientError("OpenClaw execution is disabled.")

        command = [
            self.binary,
            "agent",
            "--agent",
            self.agent_id,
            "--session-id",
            self.session_id,
            "--message",
            prompt,
            "--thinking",
            "off",
            "--json",
            "--timeout",
            str(self.timeout_seconds),
        ]

        completed = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
        )
        if completed.returncode != 0:
            message = completed.stderr.strip() or completed.stdout.strip() or "OpenClaw agent failed."
            raise OpenClawClientError(message)

        try:
            envelope = json.loads(completed.stdout)
        except json.JSONDecodeError as exc:
            raise OpenClawClientError(f"OpenClaw returned non-JSON output: {exc}") from exc

        payload_text = self._extract_text_payload(envelope)
        try:
            return json.loads(self._extract_json(payload_text))
        except json.JSONDecodeError as exc:
            raise OpenClawClientError(f"OpenClaw returned malformed JSON payload: {exc}") from exc

    @staticmethod
    def _extract_text_payload(envelope: dict[str, Any]) -> str:
        result = envelope.get("result") or {}
        payloads = result.get("payloads") or []
        texts = [payload.get("text", "") for payload in payloads if isinstance(payload, dict)]
        joined = "\n".join(text.strip() for text in texts if text and text.strip())
        if joined:
            return joined
        summary = envelope.get("summary")
        if isinstance(summary, str) and summary.strip():
            return summary
        raise OpenClawClientError("OpenClaw returned no assistant payload.")

    @staticmethod
    def _extract_json(raw_text: str) -> str:
        start = raw_text.find("{")
        end = raw_text.rfind("}")
        if start == -1 or end == -1 or end < start:
            raise OpenClawClientError("OpenClaw assistant reply did not contain a JSON object.")
        return raw_text[start : end + 1]
