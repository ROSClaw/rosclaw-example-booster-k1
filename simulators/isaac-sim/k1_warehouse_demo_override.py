#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path
import copy
import json
import os
import sys
import threading
import time
import traceback

import numpy as np
import torch

SRC_DIR = Path(__file__).resolve().parents[1] / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

PORTED_RUNTIME_ROOT = Path(
    os.environ.get(
        "K1_PORTED_RUNTIME_ROOT",
        "/workspace/rosclaw-example-booster-k1/isaac-sim-runtime",
    )
)
for extra_path in (
    PORTED_RUNTIME_ROOT / "vendor" / "booster_assets" / "src",
    PORTED_RUNTIME_ROOT / "vendor" / "booster_train" / "source" / "booster_train",
):
    if extra_path.exists() and str(extra_path) not in sys.path:
        sys.path.insert(0, str(extra_path))

import booster_k1_sim.runtime as runtime


class VelocityCommandBuffer:
    def __init__(self, num_envs: int, device: torch.device | str = "cpu") -> None:
        self._commands = torch.zeros((num_envs, 3), dtype=torch.float32, device=device)
        self._lock = threading.Lock()

    def set_command(self, linear_x: float, linear_y: float, angular_z: float, robot_index: int = 0) -> None:
        with self._lock:
            self._commands[robot_index, 0] = linear_x
            self._commands[robot_index, 1] = linear_y
            self._commands[robot_index, 2] = angular_z

    def clear(self, robot_index: int = 0) -> None:
        with self._lock:
            self._commands[robot_index].zero_()

    def get_commands(self, device: torch.device | str) -> torch.Tensor:
        with self._lock:
            return self._commands.clone().to(device)


class CmdVelRuntimeNode(runtime.Node):
    def __init__(self, settings: runtime.RuntimeSettings, command_buffer: VelocityCommandBuffer):
        super().__init__(f"{settings.node_prefix}_humanoid_controller")
        self.settings = settings
        self.command_buffer = command_buffer
        self._last_cmd_vel = runtime.Twist()
        self._last_cmd_vel_at = 0.0
        self._last_cmd_vel_log_at = 0.0
        self._last_publish_time_sec: float | None = None
        self._stale_timeout_sec = 0.5
        self._status_payload = {
            "mode": "isaaclab_velocity_env",
            "namespace": settings.topic("").rstrip("/") or "/",
            "policy_path": settings.policy_path,
            "scene_usd": settings.scene_usd,
            "scene_prim_path": "/World/Warehouse",
            "control_period_ms": settings.publish_period_ms,
            "physics_steps_per_second": runtime.DEFAULT_PHYSICS_STEPS_PER_SECOND,
            "gpu_dynamics_enabled": False,
            "broadphase_type": runtime.DEFAULT_BROADPHASE_TYPE,
            "observation_source": "full_body_env_with_flat_height_scan",
            "robot_asset": "BOOSTER_K1_CFG(no_delay)",
            "terrain": "flat",
        }
        self.create_subscription(runtime.Twist, settings.topic("cmd_vel"), self._cmd_vel_cb, 10)
        self._joint_command_pub = self.create_publisher(runtime.JointState, settings.topic("joint_command"), 10)
        self._status_pub = self.create_publisher(runtime.String, settings.topic("booster/status"), 10)

    def _cmd_vel_cb(self, msg: runtime.Twist) -> None:
        self._last_cmd_vel = msg
        self._last_cmd_vel_at = time.monotonic()
        self.command_buffer.set_command(msg.linear.x, msg.linear.y, msg.angular.z)
        print(
            "[runtime] received /cmd_vel "
            f"linear=({msg.linear.x:.3f}, {msg.linear.y:.3f}) angular_z={msg.angular.z:.3f}",
            flush=True,
        )

    def _clear_if_stale(self) -> None:
        if self._last_cmd_vel_at <= 0.0:
            return
        if (time.monotonic() - self._last_cmd_vel_at) >= self._stale_timeout_sec:
            self._last_cmd_vel = runtime.Twist()
            self._last_cmd_vel_at = 0.0
            self.command_buffer.clear()

    def publish(self, env, sim_time_sec: float) -> None:
        self._clear_if_stale()
        publish_period_sec = max(self.settings.publish_period_ms, 1) / 1000.0
        if self._last_publish_time_sec is not None and (sim_time_sec - self._last_publish_time_sec) < publish_period_sec:
            return

        action_term = env.action_manager.get_term("joint_pos")
        processed_actions = action_term.processed_actions[0].detach().cpu().numpy().astype(np.float32)

        joint_command = runtime.JointState()
        joint_command.header = runtime.Header(stamp=runtime.stamp_from_sim_time(sim_time_sec))
        joint_command.name = list(action_term._joint_names)
        joint_command.position = processed_actions.tolist()
        joint_command.velocity = [0.0] * len(joint_command.name)
        joint_command.effort = [0.0] * len(joint_command.name)
        self._joint_command_pub.publish(joint_command)

        self._status_payload["action_joint_order"] = list(action_term._joint_names)
        self._status_payload["processed_action_norm"] = float(np.linalg.norm(processed_actions))
        self._status_payload["last_cmd_vel"] = {
            "linear_x": float(self._last_cmd_vel.linear.x),
            "linear_y": float(self._last_cmd_vel.linear.y),
            "angular_z": float(self._last_cmd_vel.angular.z),
        }
        status = runtime.String()
        status.data = json.dumps(self._status_payload, separators=(",", ":"))
        self._status_pub.publish(status)
        self._last_publish_time_sec = sim_time_sec


class KinematicGaitRuntimeNode(runtime.Node):
    def __init__(self, settings: runtime.RuntimeSettings, command_buffer: VelocityCommandBuffer):
        super().__init__(f"{settings.node_prefix}_kinematic_gait_controller")
        self.settings = settings
        self.command_buffer = command_buffer
        self._last_cmd_vel = runtime.Twist()
        self._last_cmd_vel_at = 0.0
        self._last_cmd_vel_log_at = 0.0
        self._last_publish_time_sec: float | None = None
        self._stale_timeout_sec = 0.5
        lin_x_min = float(os.environ.get("K1_KINEMATIC_LIN_X_MIN", "-0.8"))
        lin_x_max = float(os.environ.get("K1_KINEMATIC_LIN_X_MAX", "0.8"))
        self._lin_x_range = (min(lin_x_min, lin_x_max), max(lin_x_min, lin_x_max))
        self._lin_y_range = runtime.TRAINED_CMD_LIN_Y_RANGE
        self._ang_z_range = runtime.TRAINED_CMD_ANG_Z_RANGE
        self._joint_command_pub = self.create_publisher(runtime.JointState, settings.topic("joint_command"), 10)
        self._status_pub = self.create_publisher(runtime.String, settings.topic("booster/status"), 10)
        self._status_payload = {
            "mode": "kinematic_gait",
            "namespace": settings.topic("").rstrip("/") or "/",
            "policy_path": settings.policy_path,
            "scene_usd": settings.scene_usd,
            "scene_prim_path": "/World/Warehouse",
            "control_period_ms": settings.publish_period_ms,
            "physics_steps_per_second": runtime.DEFAULT_PHYSICS_STEPS_PER_SECOND,
            "gpu_dynamics_enabled": False,
            "broadphase_type": runtime.DEFAULT_BROADPHASE_TYPE,
            "controller": "scripted_upright_root_with_leg_gait",
            "controller_note": "Deterministic fallback because current K1 RL checkpoint falls in canonical play.",
            "cmd_linear_x_range": self._lin_x_range,
        }
        self.create_subscription(runtime.Twist, settings.topic("cmd_vel"), self._cmd_vel_cb, 10)

    def _cmd_vel_cb(self, msg: runtime.Twist) -> None:
        clipped_x = float(np.clip(msg.linear.x, *self._lin_x_range))
        clipped_y = float(np.clip(msg.linear.y, *self._lin_y_range))
        clipped_yaw = float(np.clip(msg.angular.z, *self._ang_z_range))
        self._last_cmd_vel = runtime.Twist()
        self._last_cmd_vel.linear.x = clipped_x
        self._last_cmd_vel.linear.y = clipped_y
        self._last_cmd_vel.angular.z = clipped_yaw
        now = time.monotonic()
        self._last_cmd_vel_at = now
        self.command_buffer.set_command(clipped_x, clipped_y, clipped_yaw)
        if now - self._last_cmd_vel_log_at >= 0.5:
            print(
                "[runtime] received /cmd_vel "
                f"linear=({msg.linear.x:.3f}, {msg.linear.y:.3f}) angular_z={msg.angular.z:.3f}; "
                f"clipped=({clipped_x:.3f}, {clipped_y:.3f}, {clipped_yaw:.3f})",
                flush=True,
            )
            self._last_cmd_vel_log_at = now

    def _clear_if_stale(self) -> None:
        if self._last_cmd_vel_at <= 0.0:
            return
        if (time.monotonic() - self._last_cmd_vel_at) >= self._stale_timeout_sec:
            self._last_cmd_vel = runtime.Twist()
            self._last_cmd_vel_at = 0.0
            self.command_buffer.clear()

    def publish(self, robot, joint_targets: torch.Tensor, sim_time_sec: float) -> None:
        self._clear_if_stale()
        publish_period_sec = max(self.settings.publish_period_ms, 1) / 1000.0
        if self._last_publish_time_sec is not None and (sim_time_sec - self._last_publish_time_sec) < publish_period_sec:
            return

        joint_command = runtime.JointState()
        joint_command.header = runtime.Header(stamp=runtime.stamp_from_sim_time(sim_time_sec))
        joint_command.name = list(robot.joint_names)
        joint_command.position = joint_targets[0].detach().cpu().numpy().astype(np.float32).tolist()
        joint_command.velocity = [0.0] * len(joint_command.name)
        joint_command.effort = [0.0] * len(joint_command.name)
        self._joint_command_pub.publish(joint_command)

        command = self.command_buffer.get_commands("cpu")[0]
        root_state = robot.data.root_state_w[0].detach().cpu()
        self._status_payload["last_cmd_vel"] = {
            "linear_x": float(command[0].item()),
            "linear_y": float(command[1].item()),
            "angular_z": float(command[2].item()),
        }
        self._status_payload["root_position"] = {
            "x": float(root_state[0].item()),
            "y": float(root_state[1].item()),
            "z": float(root_state[2].item()),
        }
        self._status_payload["root_orientation_wxyz"] = [float(root_state[i].item()) for i in range(3, 7)]
        status = runtime.String()
        status.data = json.dumps(self._status_payload, separators=(",", ":"))
        self._status_pub.publish(status)
        self._last_publish_time_sec = sim_time_sec


class KinematicGaitController:
    def __init__(self, robot, settings: runtime.RuntimeSettings, device) -> None:
        self.robot = robot
        self.settings = settings
        self.device = device
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.phase = 0.0
        self.speed_scale = float(os.environ.get("K1_KINEMATIC_SPEED_SCALE", "1.0"))
        self.nominal_joint_pos = robot.data.default_joint_pos.clone()
        self.nominal_joint_vel = torch.zeros_like(robot.data.default_joint_vel)
        for index, name in enumerate(robot.joint_names):
            if name == "Left_Shoulder_Roll":
                self.nominal_joint_pos[:, index] = -1.3
            elif name == "Right_Shoulder_Roll":
                self.nominal_joint_pos[:, index] = 1.3
            elif "_Hip_Pitch" in name:
                self.nominal_joint_pos[:, index] = -0.2
            elif "_Knee_Pitch" in name:
                self.nominal_joint_pos[:, index] = 0.4
            elif "_Ankle_Pitch" in name:
                self.nominal_joint_pos[:, index] = -0.25
        self._joint_index_by_name = {name: index for index, name in enumerate(robot.joint_names)}
        self._root_state = robot.data.default_root_state.clone()
        self._root_state[:, :3] = torch.tensor((0.0, 0.0, settings.robot_z), dtype=torch.float32, device=device)
        self._root_state[:, 3:7] = torch.tensor((1.0, 0.0, 0.0, 0.0), dtype=torch.float32, device=device)
        self._root_state[:, 7:] = 0.0

    def _set_joint(self, joint_targets: torch.Tensor, name: str, value: float) -> None:
        index = self._joint_index_by_name.get(name)
        if index is not None:
            joint_targets[:, index] = value

    def _yaw_quat(self) -> torch.Tensor:
        half_yaw = 0.5 * self.yaw
        return torch.tensor(
            (np.cos(half_yaw), 0.0, 0.0, np.sin(half_yaw)),
            dtype=torch.float32,
            device=self.device,
        )

    def step(self, command: torch.Tensor, dt: float) -> tuple[torch.Tensor, torch.Tensor]:
        vx = float(command[0].item()) * self.speed_scale
        vy = float(command[1].item()) * self.speed_scale
        wz = float(command[2].item()) * self.speed_scale
        speed = float(np.clip(np.hypot(vx, vy), 0.0, 1.0))
        active = speed > 0.015 or abs(wz) > 0.015

        cos_yaw = np.cos(self.yaw)
        sin_yaw = np.sin(self.yaw)
        world_vx = cos_yaw * vx - sin_yaw * vy
        world_vy = sin_yaw * vx + cos_yaw * vy
        self.x += world_vx * dt
        self.y += world_vy * dt
        self.yaw += wz * dt

        frequency_hz = 1.15 + 0.85 * speed
        if active:
            self.phase += 2.0 * np.pi * frequency_hz * dt
        else:
            # Return toward a symmetric stance when commands stop.
            self.phase *= max(0.0, 1.0 - 4.0 * dt)

        gait_scale = float(np.clip(speed / 0.55, 0.0, 1.0))
        yaw_scale = float(np.clip(abs(wz) / runtime.TRAINED_CMD_ANG_Z_RANGE[1], 0.0, 1.0))
        stride = 0.34 * gait_scale + 0.12 * yaw_scale
        knee_lift = 0.22 * gait_scale + 0.08 * yaw_scale
        ankle = 0.16 * gait_scale + 0.05 * yaw_scale
        lateral = float(np.clip(vy / max(abs(runtime.TRAINED_CMD_LIN_Y_RANGE[0]), 1.0e-6), -1.0, 1.0))
        turn_bias = float(np.clip(wz / runtime.TRAINED_CMD_ANG_Z_RANGE[1], -1.0, 1.0))

        left = np.sin(self.phase)
        right = -left
        left_lift = max(0.0, left)
        right_lift = max(0.0, right)
        joint_targets = self.nominal_joint_pos.clone()

        self._set_joint(joint_targets, "Left_Hip_Pitch", -0.2 + stride * left)
        self._set_joint(joint_targets, "Right_Hip_Pitch", -0.2 + stride * right)
        self._set_joint(joint_targets, "Left_Knee_Pitch", 0.4 + knee_lift * left_lift)
        self._set_joint(joint_targets, "Right_Knee_Pitch", 0.4 + knee_lift * right_lift)
        self._set_joint(joint_targets, "Left_Ankle_Pitch", -0.25 - ankle * left)
        self._set_joint(joint_targets, "Right_Ankle_Pitch", -0.25 - ankle * right)
        self._set_joint(joint_targets, "Left_Hip_Roll", 0.06 * lateral + 0.05 * turn_bias)
        self._set_joint(joint_targets, "Right_Hip_Roll", 0.06 * lateral - 0.05 * turn_bias)
        self._set_joint(joint_targets, "Left_Ankle_Roll", -0.04 * lateral - 0.04 * turn_bias)
        self._set_joint(joint_targets, "Right_Ankle_Roll", -0.04 * lateral + 0.04 * turn_bias)
        self._set_joint(joint_targets, "Left_Hip_Yaw", 0.05 * turn_bias)
        self._set_joint(joint_targets, "Right_Hip_Yaw", -0.05 * turn_bias)

        bob = 0.015 * gait_scale * max(0.0, np.sin(2.0 * self.phase))
        self._root_state[:, 0] = self.x
        self._root_state[:, 1] = self.y
        self._root_state[:, 2] = self.settings.robot_z + bob
        self._root_state[:, 3:7] = self._yaw_quat()
        self._root_state[:, 7] = world_vx
        self._root_state[:, 8] = world_vy
        self._root_state[:, 9] = 0.0
        self._root_state[:, 10] = 0.0
        self._root_state[:, 11] = 0.0
        self._root_state[:, 12] = wz

        self.robot.write_root_state_to_sim(self._root_state)
        self.robot.write_joint_state_to_sim(joint_targets, self.nominal_joint_vel)
        return self._root_state, joint_targets


def _wait_for_app_running(simulation_app, timeout_sec: float = 30.0) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if simulation_app.is_running():
            return True
        simulation_app.update()
    return simulation_app.is_running()


def _resolve_scene_reference(scene_usd: str) -> str:
    if not scene_usd:
        return ""
    if "://" in scene_usd:
        return scene_usd
    try:
        from isaacsim.storage.native import get_full_asset_path

        resolved = get_full_asset_path(scene_usd)
        if resolved:
            return resolved
    except Exception:
        pass
    return scene_usd


def _configure_stream_view(settings) -> None:
    try:
        from pxr import Gf, Sdf, UsdGeom
        import omni.usd
        import omni.kit.viewport.utility as viewport_utility

        view_mode = os.environ.get("K1_VIEW_CAMERA_MODE", "observer").strip().lower()
        target_path = settings.camera_prim_path
        if view_mode == "observer":
            stage = omni.usd.get_context().get_stage()
            target_path = os.environ.get("K1_VIEW_CAMERA_PATH", "/World/K1ObserverCamera")
            camera_prim = stage.GetPrimAtPath(target_path)
            if not camera_prim.IsValid():
                camera = UsdGeom.Camera.Define(stage, target_path)
                camera.GetFocalLengthAttr().Set(float(os.environ.get("K1_VIEW_CAMERA_FOCAL_LENGTH", "28.0")))
                camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 1.0e5))
            try:
                from isaacsim.core.utils.viewports import set_camera_view

                eye = tuple(float(v) for v in os.environ.get("K1_VIEW_CAMERA_EYE", "2.6,-3.2,1.35").split(","))
                target = tuple(float(v) for v in os.environ.get("K1_VIEW_CAMERA_TARGET", "0.0,0.0,0.55").split(","))
                set_camera_view(eye=eye, target=target, camera_prim_path=target_path)
            except Exception as exc:
                print(f"[runtime] observer camera view helper failed: {exc}", flush=True)
        elif view_mode in {"front", "front_cam", "robot_front"}:
            target_path = settings.camera_prim_path
        else:
            target_path = view_mode

        active_viewport = None
        if hasattr(viewport_utility, "get_active_viewport"):
            active_viewport = viewport_utility.get_active_viewport()
        if active_viewport is None and hasattr(viewport_utility, "get_active_viewport_window"):
            viewport_window = viewport_utility.get_active_viewport_window()
            active_viewport = getattr(viewport_window, "viewport_api", None) if viewport_window is not None else None

        if active_viewport is None:
            print("[runtime] no active viewport API found; leaving default camera unchanged", flush=True)
            return

        if hasattr(active_viewport, "set_active_camera"):
            active_viewport.set_active_camera(target_path)
        elif hasattr(active_viewport, "camera_path"):
            try:
                active_viewport.camera_path = Sdf.Path(target_path)
            except Exception:
                active_viewport.camera_path = target_path
        else:
            print("[runtime] active viewport API has no camera setter; leaving default camera unchanged", flush=True)
            return
        print(f"[runtime] active viewport camera set to {target_path}", flush=True)
    except Exception as exc:
        print(f"[runtime] failed to set active viewport camera: {exc}", flush=True)


def _write_ready_file(settings) -> None:
    ready_path = Path(os.environ.get("ROSCLAW_READY_FILE", "/tmp/rosclaw-k1-ready.json"))
    payload = {
        "namespace": settings.topic("").rstrip("/") or "/",
        "cmdVelTopic": settings.topic("cmd_vel"),
        "jointCommandTopic": settings.topic("joint_command"),
        "odomTopic": settings.topic("odom"),
        "imuTopic": settings.topic("imu"),
        "jointStateTopic": settings.topic("joint_states"),
        "cameraTopic": settings.topic("front_cam/rgb"),
        "cameraInfoTopic": settings.topic("front_cam/camera_info"),
        "statusTopic": settings.topic("booster/status"),
        "sceneUsd": settings.scene_usd,
        "policyPath": settings.policy_path,
        "controlPeriodMs": settings.publish_period_ms,
        "livestreamPort": int(os.environ.get("ISAAC_LIVESTREAM_PORT", "49100")),
        "rosbridgeUrl": "ws://localhost:9090",
    }
    ready_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(f"[runtime] wrote readiness file to {ready_path}", flush=True)


def _write_state_file(
    settings,
    env,
    command_buffer: VelocityCommandBuffer,
    sim_time_sec: float,
    action_norm: float,
    processed_action_norm: float,
    policy_obs: torch.Tensor | None = None,
) -> None:
    state_path = Path(os.environ.get("K1_STATE_FILE", "/tmp/rosclaw-k1-state.json"))
    robot = env.scene["robot"]
    root_state = robot.data.root_state_w[0]
    command = command_buffer.get_commands("cpu")[0]
    command_term = env.command_manager.get_term("base_velocity")
    command_term_value = command_term.command[0].detach().cpu()
    body_linear_velocity = robot.data.root_lin_vel_b[0].detach().cpu()
    payload = {
        "simTimeSec": float(sim_time_sec),
        "rootPosition": {
            "x": float(root_state[0].item()),
            "y": float(root_state[1].item()),
            "z": float(root_state[2].item()),
        },
        "rootOrientation": {
            "w": float(root_state[3].item()),
            "x": float(root_state[4].item()),
            "y": float(root_state[5].item()),
            "z": float(root_state[6].item()),
        },
        "rootLinearVelocity": {
            "x": float(root_state[7].item()),
            "y": float(root_state[8].item()),
            "z": float(root_state[9].item()),
        },
        "bodyLinearVelocity": {
            "x": float(body_linear_velocity[0].item()),
            "y": float(body_linear_velocity[1].item()),
            "z": float(body_linear_velocity[2].item()),
        },
        "rootAngularVelocity": {
            "x": float(root_state[10].item()),
            "y": float(root_state[11].item()),
            "z": float(root_state[12].item()),
        },
        "lastCmdVel": {
            "linearX": float(command[0].item()),
            "linearY": float(command[1].item()),
            "angularZ": float(command[2].item()),
        },
        "commandManagerCommand": {
            "linearX": float(command_term_value[0].item()),
            "linearY": float(command_term_value[1].item()),
            "angularZ": float(command_term_value[2].item()),
        },
        "actionNorm": float(action_norm),
        "processedActionNorm": float(processed_action_norm),
    }
    if policy_obs is not None:
        flat_obs = policy_obs[0].detach().cpu()
        payload["policyCommandObservation"] = {
            "linearX": float(flat_obs[9].item()),
            "linearY": float(flat_obs[10].item()),
            "angularZ": float(flat_obs[11].item()),
        }
        payload["policyHeightScanMean"] = float(flat_obs[68:].mean().item())
        payload["policyHeightScanAbsMax"] = float(flat_obs[68:].abs().max().item())
    state_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _write_kinematic_state_file(settings, robot, command_buffer: VelocityCommandBuffer, sim_time_sec: float) -> None:
    state_path = Path(os.environ.get("K1_STATE_FILE", "/tmp/rosclaw-k1-state.json"))
    root_state = robot.data.root_state_w[0].detach().cpu()
    command = command_buffer.get_commands("cpu")[0]
    payload = {
        "simTimeSec": float(sim_time_sec),
        "controllerMode": "kinematic_gait",
        "rootPosition": {
            "x": float(root_state[0].item()),
            "y": float(root_state[1].item()),
            "z": float(root_state[2].item()),
        },
        "rootOrientation": {
            "w": float(root_state[3].item()),
            "x": float(root_state[4].item()),
            "y": float(root_state[5].item()),
            "z": float(root_state[6].item()),
        },
        "rootLinearVelocity": {
            "x": float(root_state[7].item()),
            "y": float(root_state[8].item()),
            "z": float(root_state[9].item()),
        },
        "rootAngularVelocity": {
            "x": float(root_state[10].item()),
            "y": float(root_state[11].item()),
            "z": float(root_state[12].item()),
        },
        "lastCmdVel": {
            "linearX": float(command[0].item()),
            "linearY": float(command[1].item()),
            "angularZ": float(command[2].item()),
        },
        "jointCount": len(robot.joint_names),
        "robotZ": float(settings.robot_z),
    }
    state_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _get_sim_time_sec(env) -> float:
    current_time = getattr(env.sim, "current_time", None)
    if current_time is not None:
        return float(current_time)

    step_dt = getattr(env, "step_dt", None)
    step_index = getattr(env, "common_step_counter", 0)
    if step_dt is not None:
        return float(step_index) * float(step_dt)
    return 0.0


def _prepare_policy_obs(policy_obs: torch.Tensor) -> torch.Tensor:
    obs_mode = os.environ.get("K1_POLICY_OBS_MODE", "flat_height_scan").strip().lower()
    command_scale = float(os.environ.get("K1_POLICY_COMMAND_SCALE", "1.0"))
    command_offset = float(os.environ.get("K1_POLICY_COMMAND_OFFSET", "0.0"))
    target_obs_dim = int(os.environ.get("K1_POLICY_TARGET_OBS_DIM", os.environ.get("K1_POLICY_OBS_DIM", "255")))

    if policy_obs.shape[-1] == 255:
        prepared = policy_obs.clone()
        if obs_mode == "flat_height_scan":
            prepared[:, 68:] = 0.0
        elif obs_mode == "raw":
            pass
        else:
            raise RuntimeError(f"Unsupported K1_POLICY_OBS_MODE={obs_mode!r} for 255-dim observations")
        if command_scale != 1.0 or command_offset != 0.0:
            prepared[:, 9:12] = prepared[:, 9:12] * command_scale + command_offset
        return prepared
    if policy_obs.shape[-1] != 235:
        raise RuntimeError(f"Unsupported policy observation shape {tuple(policy_obs.shape)}")

    if target_obs_dim == 235:
        prepared = policy_obs.clone()
        if obs_mode == "flat_height_scan":
            prepared[:, 48:] = 0.0
        elif obs_mode == "raw":
            pass
        else:
            raise RuntimeError(f"Unsupported K1_POLICY_OBS_MODE={obs_mode!r} for 235-dim observations")
        if command_scale != 1.0 or command_offset != 0.0:
            prepared[:, 9:12] = prepared[:, 9:12] * command_scale + command_offset
        return prepared

    base_terms = policy_obs[:, :12]
    leg_joint_pos = policy_obs[:, 12:24]
    leg_joint_vel = policy_obs[:, 24:36]
    previous_actions = policy_obs[:, 36:48]
    height_scan = torch.zeros_like(policy_obs[:, 48:])
    zeros = torch.zeros((policy_obs.shape[0], 10), dtype=policy_obs.dtype, device=policy_obs.device)
    prepared = torch.cat(
        [
            base_terms,
            leg_joint_pos,
            zeros,
            leg_joint_vel,
            zeros,
            previous_actions,
            height_scan,
        ],
        dim=-1,
    )
    if command_scale != 1.0 or command_offset != 0.0:
        prepared[:, 9:12] = prepared[:, 9:12] * command_scale + command_offset
    return prepared


def _apply_nominal_standing_state(env, settings):
    robot = env.scene["robot"]
    device = env.device

    root_pose = robot.data.root_state_w[:, :7].clone()
    root_pose[:, 0] = 0.0
    root_pose[:, 1] = 0.0
    root_pose[:, 2] = float(getattr(settings, "robot_z", 0.57))
    root_pose[:, 3] = 1.0
    root_pose[:, 4:] = 0.0

    root_velocity = torch.zeros((env.num_envs, 6), dtype=torch.float32, device=device)
    joint_pos = torch.zeros_like(robot.data.joint_pos)
    joint_vel = torch.zeros_like(robot.data.joint_vel)

    for joint_index, joint_name in enumerate(robot.joint_names):
        if "_Hip_Pitch" in joint_name:
            joint_pos[:, joint_index] = -0.2
        elif "_Knee_Pitch" in joint_name:
            joint_pos[:, joint_index] = 0.4
        elif "_Ankle_Pitch" in joint_name:
            joint_pos[:, joint_index] = -0.25
        elif joint_name == "Left_Shoulder_Roll":
            joint_pos[:, joint_index] = -1.3
        elif joint_name == "Right_Shoulder_Roll":
            joint_pos[:, joint_index] = 1.3

    robot.write_root_pose_to_sim(root_pose)
    robot.write_root_velocity_to_sim(root_velocity)
    robot.write_joint_state_to_sim(joint_pos, joint_vel)
    env.scene.write_data_to_sim()
    for _ in range(4):
        env.sim.step()
        env.scene.update(env.sim.get_physics_dt())
    env.obs_buf = env.observation_manager.compute(update_history=True)
    print(
        "[runtime] applied nominal standing state "
        f"at z={root_pose[0, 2].item():.3f} for {len(robot.joint_names)} joints",
        flush=True,
    )
    return env.obs_buf


def _load_policy(env, policy_path: str):
    policy_load_mode = os.environ.get("K1_POLICY_LOAD_MODE", "auto").strip().lower()
    if policy_load_mode not in {"auto", "torchscript", "runner"}:
        raise RuntimeError(f"Unsupported K1_POLICY_LOAD_MODE={policy_load_mode!r}")

    if policy_load_mode != "runner":
        try:
            policy = torch.jit.load(policy_path, map_location=env.device)
            policy = policy.to(env.device)
            policy.eval()
            return policy, "torchscript"
        except Exception as exc:
            if policy_load_mode == "torchscript":
                raise
            print(
                f"[runtime] torchscript load failed for {policy_path}: {exc}. "
                "Falling back to RSL-RL checkpoint loader.",
                flush=True,
            )

    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
    from rsl_rl.runners import OnPolicyRunner

    from booster_train.tasks.manager_based.velocity.robots.k1.ppo_cfg import PPORunnerCfg

    runner_cfg = PPORunnerCfg()
    vec_env = RslRlVecEnvWrapper(env, clip_actions=runner_cfg.clip_actions)
    runner = OnPolicyRunner(vec_env, runner_cfg.to_dict(), log_dir=None, device=env.device)
    runner.load(policy_path)
    return runner.get_inference_policy(device=env.device), "runner"


def main(argv: list[str] | None = None) -> None:
    parser = runtime.build_arg_parser()
    args = parser.parse_args(argv)
    simulation_app = runtime.launch_app(args)
    settings = runtime.build_settings(args)

    import rclpy
    from isaaclab.envs import ManagerBasedRLEnv

    try:
        from booster_train.assets.robots.booster import BOOSTER_K1_LOCOMOTION_CFG as BUNDLED_K1_LOCOMOTION_CFG
        from booster_train.assets.robots.booster import BOOSTER_K1_ZED_CFG as BUNDLED_K1_ZED_CFG
        from booster_train.assets.robots.booster import K1_LOCOMOTION_JOINT_NAMES
        from booster_train.tasks.manager_based.velocity.mdp.command_terms import ExternalVelocityCommand
        from booster_train.tasks.manager_based.velocity.robots.k1.rough_env_cfg import K1RoughEnvCfg_PLAY
        from booster_k1_sim.assets.booster import BOOSTER_K1_CFG as BUNDLED_K1_CFG
    except Exception as exc:
        raise RuntimeError(
            "The mounted K1 override requires the ported isaac-sim-runtime tree to be mounted "
            f"at {PORTED_RUNTIME_ROOT}."
        ) from exc

    def _patched_resample_command(self, env_ids):
        if len(env_ids) == 0:
            return
        if not torch.is_tensor(env_ids):
            env_id_values = list(env_ids)
        else:
            env_id_values = env_ids.detach().cpu().tolist()
        env_ids = torch.tensor(env_id_values, dtype=torch.long, device=self._command.device)

        samples = torch.empty((len(env_ids), 3), device=self._command.device)
        samples[:, 0].uniform_(*self.cfg.ranges.lin_vel_x)
        samples[:, 1].uniform_(*self.cfg.ranges.lin_vel_y)
        samples[:, 2].uniform_(*self.cfg.ranges.ang_vel_z)
        self._command[env_ids] = samples

        if self.cfg.rel_standing_envs > 0.0:
            standing_samples = torch.rand(len(env_ids), device=self._command.device) <= self.cfg.rel_standing_envs
            self.is_standing_env[env_ids] = standing_samples
        else:
            self.is_standing_env[env_ids] = False

        self.command_age[env_ids] = 0.0
        self.no_progress_time[env_ids] = 0.0

    ExternalVelocityCommand._resample_command = _patched_resample_command

    controller_mode = os.environ.get("K1_CONTROLLER_MODE", "policy").strip().lower()
    if controller_mode in {"kinematic", "kinematic_gait", "scripted_gait"}:
        import isaaclab.sim as sim_utils
        import rclpy
        from isaaclab.assets import ArticulationCfg, AssetBaseCfg
        from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
        from isaaclab.sim import SimulationContext
        from isaaclab.utils import configclass
        from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

        @configclass
        class KinematicWarehouseSceneCfg(InteractiveSceneCfg):
            ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg())
            sky_light = AssetBaseCfg(
                prim_path="/World/skyLight",
                spawn=sim_utils.DomeLightCfg(
                    intensity=750.0,
                    texture_file=(
                        f"{ISAAC_NUCLEUS_DIR}/Materials/Textures/Skies/PolyHaven/"
                        "kloofendal_43d_clear_puresky_4k.hdr"
                    ),
                ),
            )
            robot: ArticulationCfg = copy.deepcopy(BUNDLED_K1_CFG).replace(prim_path="{ENV_REGEX_NS}/Robot")

        rclpy.init(args=None)
        scene = None
        executor = None
        try:
            if os.environ.get("K1_DISABLE_SCENE_REFERENCE", "0") == "1":
                print("[runtime] scene reference disabled by K1_DISABLE_SCENE_REFERENCE=1", flush=True)
            else:
                scene_reference = _resolve_scene_reference(settings.scene_usd)
                runtime.add_scene_reference(scene_reference)
                print(f"[runtime] scene reference resolved to {scene_reference}", flush=True)
            simulation_app.update()

            sim = SimulationContext(
                sim_utils.SimulationCfg(
                    dt=runtime.DEFAULT_SIM_DT,
                    render_interval=max(1, settings.policy_decimation),
                    device=args.device,
                )
            )
            runtime.configure_scene_physics(sim)
            scene_cfg = KinematicWarehouseSceneCfg(num_envs=1, env_spacing=2.5)
            robot_cfg = copy.deepcopy(BUNDLED_K1_CFG).replace(prim_path="{ENV_REGEX_NS}/Robot")
            robot_cfg.init_state.pos = (0.0, 0.0, settings.robot_z)
            scene_cfg.robot = robot_cfg
            scene = InteractiveScene(scene_cfg)
            sim.reset()
            scene.update(sim.get_physics_dt())
            print("[runtime] kinematic gait scene created and physics configured", flush=True)

            runtime.add_front_camera(settings)
            runtime.create_front_camera_graph(settings)
            _configure_stream_view(settings)
            print("[runtime] front camera graph configured", flush=True)
            if not _wait_for_app_running(simulation_app):
                raise RuntimeError("Isaac Sim never entered a running state after K1 kinematic gait setup.")

            command_buffer = VelocityCommandBuffer(1, sim.device)
            gait = KinematicGaitController(scene["robot"], settings, sim.device)
            state_publisher = runtime.K1StatePublisher(settings)
            controller = KinematicGaitRuntimeNode(settings, command_buffer)
            executor = runtime.SingleThreadedExecutor()
            executor.add_node(state_publisher)
            executor.add_node(controller)
            _write_ready_file(settings)
            print("[runtime] entering kinematic gait cmd_vel loop", flush=True)

            step_count = 0
            sim_dt = float(sim.get_physics_dt())
            dt_mode = os.environ.get("K1_KINEMATIC_DT_MODE", "wall").strip().lower()
            max_motion_dt = float(os.environ.get("K1_KINEMATIC_MAX_DT", "0.05"))
            start_wall_time = time.monotonic()
            last_motion_wall_time = start_wall_time
            state_write_period = max(settings.publish_period_ms, 200) / 1000.0
            last_state_write_wall_time = 0.0
            first_loop = True
            while simulation_app.is_running():
                executor.spin_once(timeout_sec=0.0)
                command = command_buffer.get_commands(sim.device)[0]
                now = time.monotonic()
                motion_dt = sim_dt
                if dt_mode in {"wall", "realtime", "real_time"}:
                    motion_dt = min(max(now - last_motion_wall_time, sim_dt), max_motion_dt)
                last_motion_wall_time = now
                _, joint_targets = gait.step(command, motion_dt)
                scene.write_data_to_sim()
                sim.step()
                scene.update(sim_dt)
                sim_time_sec = now - start_wall_time if dt_mode in {"wall", "realtime", "real_time"} else step_count * sim_dt
                state_publisher.publish(scene["robot"], sim_time_sec)
                controller.publish(scene["robot"], joint_targets, sim_time_sec)

                if (last_state_write_wall_time <= 0.0) or ((now - last_state_write_wall_time) >= state_write_period):
                    _write_kinematic_state_file(settings, scene["robot"], command_buffer, sim_time_sec)
                    last_state_write_wall_time = now
                if first_loop:
                    print("[runtime] first kinematic gait control step completed", flush=True)
                    first_loop = False
                step_count += 1
        except BaseException as exc:
            print(f"[runtime] unhandled exception: {type(exc).__name__}: {exc}", flush=True)
            traceback.print_exc()
            raise
        finally:
            if executor is not None:
                try:
                    executor.shutdown()
                except Exception:
                    pass
            if rclpy.ok():
                rclpy.shutdown()
            simulation_app.close()
        return

    def build_env_cfg():
        env_cfg = K1RoughEnvCfg_PLAY()
        env_cfg.scene.num_envs = 1
        env_cfg.scene.env_spacing = 2.5
        env_cfg.decimation = max(settings.policy_decimation, 1)
        env_cfg.sim.dt = runtime.DEFAULT_SIM_DT
        env_cfg.sim.render_interval = max(1, settings.policy_decimation)
        env_cfg.episode_length_s = 40.0

        robot_asset_mode = os.environ.get("K1_ROBOT_ASSET_MODE", "full").strip().lower()
        if robot_asset_mode == "locomotion":
            robot_cfg = copy.deepcopy(BUNDLED_K1_LOCOMOTION_CFG)
        elif getattr(settings, "use_zed_head", False):
            robot_cfg = copy.deepcopy(BUNDLED_K1_ZED_CFG)
        else:
            robot_cfg = copy.deepcopy(BUNDLED_K1_CFG)
        robot_cfg = robot_cfg.replace(prim_path="{ENV_REGEX_NS}/Robot")
        env_cfg.scene.robot = robot_cfg
        env_cfg.scene.terrain.terrain_type = "plane"
        env_cfg.scene.terrain.terrain_generator = None
        env_cfg.scene.terrain.max_init_terrain_level = None
        env_cfg.curriculum.terrain_levels = None

        env_cfg.observations.policy.enable_corruption = False
        env_cfg.events.unique_terrain_origins = None
        env_cfg.events.reset_base = None
        env_cfg.events.reset_robot_joints = None
        env_cfg.terminations.command_stuck = None
        return env_cfg

    def _build_action_reorder_indices(env):
        action_term = env.action_manager.get_term("joint_pos")
        actual_joint_order = list(action_term._joint_names)
        expected_joint_order = list(K1_LOCOMOTION_JOINT_NAMES)
        if actual_joint_order == expected_joint_order:
            print("[runtime] policy joint order already matches action term order", flush=True)
            return None, expected_joint_order, actual_joint_order

        index_by_joint = {joint_name: idx for idx, joint_name in enumerate(expected_joint_order)}
        try:
            reorder_indices = [index_by_joint[joint_name] for joint_name in actual_joint_order]
        except KeyError as exc:
            raise RuntimeError(
                f"Unable to remap policy actions: env joint {exc.args[0]!r} is missing from expected locomotion order."
            ) from exc

        print(
            "[runtime] remapping policy actions "
            f"from {expected_joint_order} to {actual_joint_order}",
            flush=True,
        )
        return reorder_indices, expected_joint_order, actual_joint_order

    rclpy.init(args=None)
    env = None

    try:
        if os.environ.get("K1_DISABLE_SCENE_REFERENCE", "0") == "1":
            scene_reference = ""
            print("[runtime] scene reference disabled by K1_DISABLE_SCENE_REFERENCE=1", flush=True)
        else:
            scene_reference = _resolve_scene_reference(settings.scene_usd)
            runtime.add_scene_reference(scene_reference)
            print(f"[runtime] scene reference resolved to {scene_reference}", flush=True)
        env = ManagerBasedRLEnv(build_env_cfg())
        runtime.configure_scene_physics(env.sim)
        print("[runtime] warehouse stage referenced and physics configured", flush=True)

        obs, _ = env.reset()
        if os.environ.get("K1_MANUAL_STANDING_RESET", "1") == "1":
            obs = _apply_nominal_standing_state(env, settings)
            print("[runtime] env reset complete with manual standing override", flush=True)
        else:
            print("[runtime] env reset complete using vendor reset flow", flush=True)

        runtime.add_front_camera(settings)
        runtime.create_front_camera_graph(settings)
        _configure_stream_view(settings)
        print("[runtime] front camera graph configured", flush=True)
        if not _wait_for_app_running(simulation_app):
            raise RuntimeError("Isaac Sim never entered a running state after K1 cmd_vel setup.")

        state_publisher = runtime.K1StatePublisher(settings)
        command_buffer = VelocityCommandBuffer(1, env.device)
        controller = CmdVelRuntimeNode(settings, command_buffer)
        print("[runtime] ROS nodes created", flush=True)
        env.external_command_buffer = command_buffer
        remap_policy_actions = os.environ.get("K1_REMAP_POLICY_ACTIONS", "0") == "1"
        action_reorder_indices = None
        expected_joint_order = None
        actual_joint_order = list(env.action_manager.get_term("joint_pos")._joint_names)
        if remap_policy_actions:
            action_reorder_indices, expected_joint_order, actual_joint_order = _build_action_reorder_indices(env)
        else:
            print("[runtime] K1_REMAP_POLICY_ACTIONS=0, using vendor action ordering", flush=True)
        controller._status_payload["policy_action_joint_order"] = expected_joint_order
        controller._status_payload["env_action_joint_order"] = actual_joint_order

        policy, loaded_policy_mode = _load_policy(env, settings.policy_path)
        print(f"[runtime] loaded {loaded_policy_mode} policy from {settings.policy_path}", flush=True)
        force_zero_actions = os.environ.get("K1_FORCE_ZERO_ACTIONS", "0") == "1"
        action_gain = float(os.environ.get("K1_POLICY_ACTION_GAIN", "1.0"))
        if force_zero_actions:
            print("[runtime] K1_FORCE_ZERO_ACTIONS=1, overriding policy output with zeros", flush=True)
        if action_gain != 1.0:
            print(f"[runtime] scaling raw policy actions by {action_gain:.3f}", flush=True)

        executor = runtime.SingleThreadedExecutor()
        executor.add_node(state_publisher)
        executor.add_node(controller)
        _write_ready_file(settings)
        print("[runtime] entering cmd_vel control loop", flush=True)
        first_loop = True
        state_write_period = max(settings.publish_period_ms, 200) / 1000.0
        last_state_write_wall_time = 0.0

        while True:
            executor.spin_once(timeout_sec=0.0)
            policy_obs = obs["policy"] if isinstance(obs, dict) else obs
            policy_obs = policy_obs.to(env.device)
            policy_obs = _prepare_policy_obs(policy_obs)
            if force_zero_actions:
                action = torch.zeros((env.num_envs, 12), dtype=torch.float32, device=env.device)
            else:
                with torch.inference_mode():
                    action = policy(policy_obs)
            processed_action = torch.clamp(action * action_gain, -1.0, 1.0)
            if action_reorder_indices is not None:
                processed_action = processed_action[:, action_reorder_indices]
            obs, _, _, _, _ = env.step(processed_action)
            sim_time_sec = _get_sim_time_sec(env)
            state_publisher.publish(env.scene["robot"], sim_time_sec)
            controller.publish(env, sim_time_sec)
            now = time.monotonic()
            if (last_state_write_wall_time <= 0.0) or ((now - last_state_write_wall_time) >= state_write_period):
                _write_state_file(
                    settings,
                    env,
                    command_buffer,
                    sim_time_sec,
                    action_norm=float(torch.linalg.vector_norm(action[0]).item()),
                    processed_action_norm=float(torch.linalg.vector_norm(processed_action[0]).item()),
                    policy_obs=policy_obs,
                )
                last_state_write_wall_time = now
            if first_loop:
                print("[runtime] first cmd_vel control step completed", flush=True)
                first_loop = False

        executor.remove_node(controller)
        executor.remove_node(state_publisher)
        controller.destroy_node()
        state_publisher.destroy_node()
        executor.shutdown()
    except BaseException as exc:
        print(f"[runtime] unhandled exception: {type(exc).__name__}: {exc}", flush=True)
        traceback.print_exc()
        raise
    finally:
        if env is not None:
            env.close()
        if rclpy.ok():
            rclpy.shutdown()
        simulation_app.close()


runtime.main = main


if __name__ == "__main__":
    main()
