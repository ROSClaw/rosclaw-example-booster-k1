from __future__ import annotations

import json
import subprocess
import sys
from dataclasses import dataclass
from typing import Any


DATACLASS_KWARGS = {"slots": True} if sys.version_info >= (3, 10) else {}


class OpenClawClientError(RuntimeError):
    """Raised when the OpenClaw CLI fails or returns malformed payloads."""


@dataclass(**DATACLASS_KWARGS)
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
            message = self._summarize_failure_output(completed.stderr, completed.stdout)
            raise OpenClawClientError(message)

        return self._decode_response_output(completed.stdout)

    @classmethod
    def _decode_response_output(cls, stdout: str) -> dict[str, Any]:
        raw_output = stdout.strip()
        if not raw_output:
            raise OpenClawClientError("OpenClaw returned no output.")

        direct_response = cls._coerce_command_response(raw_output)
        if direct_response is not None:
            return direct_response

        json_objects = cls._parse_json_objects(raw_output)

        for candidate in reversed(json_objects):
            direct_response = cls._coerce_command_response(candidate)
            if direct_response is not None:
                return direct_response

        for envelope in reversed(json_objects):
            try:
                payload_text = cls._extract_text_payload(envelope)
            except OpenClawClientError:
                continue

            direct_response = cls._coerce_command_response(payload_text)
            if direct_response is not None:
                return direct_response

        raise OpenClawClientError("OpenClaw returned no parseable command response.")

    @staticmethod
    def _summarize_failure_output(stderr: str, stdout: str) -> str:
        combined = "\n".join(part.strip() for part in (stderr, stdout) if part and part.strip())
        if not combined:
            return "OpenClaw agent failed."

        summaries: list[str] = []

        if "duplicate plugin id detected" in combined:
            summaries.append("OpenClaw has duplicate `rosclaw` plugins configured.")
        if "plugins.allow is empty" in combined:
            summaries.append("OpenClaw is auto-loading untrusted plugins because `plugins.allow` is empty.")
        if "gateway timeout" in combined:
            summaries.append("OpenClaw gateway timed out before it could send the command.")
        if "session file locked" in combined:
            summaries.append("OpenClaw session is locked by another process or request.")

        if summaries:
            return " ".join(dict.fromkeys(summaries))

        return combined

    @classmethod
    def _parse_json_objects(cls, raw_output: str) -> list[dict[str, Any]]:
        candidates: list[dict[str, Any]] = []

        def append_candidate(value: Any) -> None:
            if isinstance(value, dict):
                candidates.append(value)
            elif isinstance(value, list):
                for item in value:
                    append_candidate(item)

        try:
            append_candidate(json.loads(raw_output))
        except json.JSONDecodeError:
            pass

        for line in raw_output.splitlines():
            stripped = line.strip()
            if not stripped:
                continue
            try:
                append_candidate(json.loads(stripped))
            except json.JSONDecodeError:
                continue

        return candidates

    @classmethod
    def _coerce_command_response(cls, value: Any) -> dict[str, Any] | None:
        payload: Any = value
        if isinstance(value, str):
            stripped = value.strip()
            if not stripped:
                return None
            try:
                payload = json.loads(stripped)
            except json.JSONDecodeError:
                try:
                    payload = json.loads(cls._extract_json(stripped))
                except (json.JSONDecodeError, OpenClawClientError):
                    return None

        if not isinstance(payload, dict):
            return None
        if "ok" not in payload or "summary" not in payload:
            return None
        return payload

    @staticmethod
    def _extract_text_payload(envelope: dict[str, Any]) -> str:
        result = envelope.get("result") or {}
        payloads = result.get("payloads") or envelope.get("payloads") or []
        texts = [payload.get("text", "") for payload in payloads if isinstance(payload, dict)]
        joined = "\n".join(text.strip() for text in texts if text and text.strip())
        if joined:
            return joined
        for key in ("text", "output_text", "message"):
            candidate = envelope.get(key)
            if isinstance(candidate, str) and candidate.strip():
                return candidate
            candidate = result.get(key)
            if isinstance(candidate, str) and candidate.strip():
                return candidate
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
