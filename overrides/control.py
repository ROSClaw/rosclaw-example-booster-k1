from __future__ import annotations

from abc import ABC, abstractmethod
from pathlib import Path
import os
import threading
import time

import numpy as np
import torch

from booster_train.assets.robots.booster import K1_LOCOMOTION_JOINT_NAMES
from booster_train.demo.runtime import RuntimeSettings


def _float_env(name: str, default: float) -> float:
    value = os.environ.get(name)
    if value is None or value == "":
        return default
    return float(value)


def _bool_env(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() not in {"0", "false", "no", "off", ""}


def _int_env(name: str, default: int) -> int:
    value = os.environ.get(name)
    if value is None or value == "":
        return default
    return int(value)


def _k1_action_scale_for_joint(joint_name: str) -> float:
    if joint_name.endswith("Hip_Pitch"):
        return 0.09375
    if joint_name.endswith("Hip_Roll"):
        return 0.109375
    if joint_name.endswith("Hip_Yaw"):
        return 0.0625
    if joint_name.endswith("Knee_Pitch"):
        return 0.125
    if joint_name.endswith("Ankle_Pitch") or joint_name.endswith("Ankle_Roll"):
        return 1.0 / 6.0
    raise KeyError(f"Unsupported K1 locomotion joint for action scaling: {joint_name}")


def _k1_default_joint_offset(joint_name: str) -> float:
    if joint_name.endswith("Hip_Pitch"):
        return -0.2
    if joint_name.endswith("Knee_Pitch"):
        return 0.4
    if joint_name.endswith("Ankle_Pitch"):
        return -0.25
    return 0.0


def _soft_deadband(values: torch.Tensor, deadband: float) -> torch.Tensor:
    if deadband <= 0.0:
        return values
    return torch.sign(values) * torch.clamp(torch.abs(values) - deadband, min=0.0)


class VelocityCommandBuffer:
    def __init__(self, num_envs: int, device: torch.device | str = "cpu"):
        self.num_envs = num_envs
        self.device = device
        self.lock = threading.Lock()
        self._commands = torch.zeros((num_envs, 3), dtype=torch.float32, device=device)
        self._desired_commands = torch.zeros((num_envs, 3), dtype=torch.float32, device=device)

    def set_command(self, robot_index: int, linear_x: float, linear_y: float, angular_z: float):
        with self.lock:
            self._desired_commands[robot_index, 0] = linear_x
            self._desired_commands[robot_index, 1] = linear_y
            self._desired_commands[robot_index, 2] = angular_z
            self._commands[robot_index, 0] = linear_x
            self._commands[robot_index, 1] = linear_y
            self._commands[robot_index, 2] = angular_z

    def set_vector(self, robot_index: int, command: tuple[float, float, float]):
        self.set_command(robot_index, command[0], command[1], command[2])

    def set_filtered_commands(self, commands: torch.Tensor):
        with self.lock:
            self._commands.copy_(commands.to(self.device))

    def clear(self, robot_index: int | None = None):
        with self.lock:
            if robot_index is None:
                self._commands.zero_()
                self._desired_commands.zero_()
            else:
                self._commands[robot_index].zero_()
                self._desired_commands[robot_index].zero_()

    def get_commands(self, device: torch.device | str) -> torch.Tensor:
        with self.lock:
            return self._commands.clone().to(device)

    def get_desired_commands(self, device: torch.device | str) -> torch.Tensor:
        with self.lock:
            return self._desired_commands.clone().to(device)


class CmdVelSubNode:
    def __init__(
        self,
        settings: RuntimeSettings,
        command_buffer: VelocityCommandBuffer,
        stale_timeout_sec: float = 0.5,
    ):
        self.settings = settings
        self.command_buffer = command_buffer
        self.stale_timeout_sec = stale_timeout_sec
        self.thread = None
        self.subscriptions = []
        self._node = None
        self._rclpy = None
        self._state_lock = threading.Lock()
        self._last_message_time = [0.0] * settings.env_count
        self._has_active_command = [False] * settings.env_count

    def _cmd_vel_cb(self, msg, robot_index: int):
        self.command_buffer.set_command(
            robot_index,
            msg.linear.x,
            msg.linear.y,
            msg.angular.z,
        )
        with self._state_lock:
            self._last_message_time[robot_index] = time.monotonic()
            self._has_active_command[robot_index] = True

    def start(self) -> bool:
        try:
            import rclpy
            from geometry_msgs.msg import Twist
            from rclpy.node import Node
        except Exception:
            return False

        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init(args=None)

        parent = self

        class _CmdVelNode(Node):
            def __init__(self):
                super().__init__(parent.settings.node_name("k1_cmd_vel_sub_node"))
                for robot_index in parent.settings.iter_robot_indices():
                    topic = parent.settings.topic("cmd_vel", robot_index)
                    sub = self.create_subscription(
                        Twist,
                        topic,
                        lambda msg, idx=robot_index: parent._cmd_vel_cb(msg, idx),
                        10,
                    )
                    parent.subscriptions.append(sub)

        self._node = _CmdVelNode()
        self.thread = threading.Thread(target=rclpy.spin, args=(self._node,), daemon=True)
        self.thread.start()
        return True

    def update(self):
        if self.stale_timeout_sec <= 0.0:
            return

        now = time.monotonic()
        stale_robot_indices: list[int] = []
        with self._state_lock:
            for robot_index, last_message_time in enumerate(self._last_message_time):
                if self._has_active_command[robot_index] and (now - last_message_time) >= self.stale_timeout_sec:
                    self._has_active_command[robot_index] = False
                    stale_robot_indices.append(robot_index)

        for robot_index in stale_robot_indices:
            self.command_buffer.clear(robot_index)

    def destroy_node(self):
        if self._node is not None:
            self._node.destroy_node()
            self._node = None

    def shutdown(self):
        self.destroy_node()

        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None

        if self._rclpy is not None and self._rclpy.ok():
            try:
                self._rclpy.shutdown()
            except Exception:
                pass
        self._rclpy = None


class TeleopSource(ABC):
    @abstractmethod
    def update_command_buffer(self, command_buffer: VelocityCommandBuffer) -> None:
        raise NotImplementedError

    def shutdown(self):
        pass


class _CmdVelTeleopSource(TeleopSource):
    def __init__(self, settings: RuntimeSettings, command_buffer: VelocityCommandBuffer):
        import omni.graph.core as og

        self._output_attributes: list[tuple[object, object]] = []
        self._og = og

        keys = og.Controller.Keys
        for robot_index in settings.iter_robot_indices():
            graph_path = settings.graph_path("K1CmdVelTeleop", robot_index)
            og.Controller.edit(
                {
                    "graph_path": graph_path,
                    "evaluator_name": "execution",
                    "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
                },
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                    ],
                    keys.SET_VALUES: [
                        ("SubscribeTwist.inputs:queueSize", 1),
                        ("SubscribeTwist.inputs:topicName", settings.topic("cmd_vel", robot_index)),
                    ],
                },
            )
            self._output_attributes.append(
                (
                    og.Controller.attribute(f"{graph_path}/SubscribeTwist.outputs:linearVelocity"),
                    og.Controller.attribute(f"{graph_path}/SubscribeTwist.outputs:angularVelocity"),
                )
            )

    def update_command_buffer(self, command_buffer: VelocityCommandBuffer) -> None:
        for robot_index, (linear_attr, angular_attr) in enumerate(self._output_attributes):
            linear_velocity = linear_attr.get()
            angular_velocity = angular_attr.get()

            if linear_velocity is None or angular_velocity is None:
                command_buffer.clear(robot_index)
                continue

            command_buffer.set_command(
                robot_index,
                float(linear_velocity[0]),
                float(linear_velocity[1]),
                float(angular_velocity[2]),
            )

    def shutdown(self):
        self._output_attributes.clear()


class _IsaacLabSe2Device(TeleopSource):
    def __init__(self, device_name: str, sim_device: torch.device | str):
        self._device = None
        self._last_active = False

        if device_name == "keyboard":
            from isaaclab.devices import Se2Keyboard, Se2KeyboardCfg

            self._device = Se2Keyboard(Se2KeyboardCfg(sim_device=str(sim_device)))
        elif device_name == "gamepad":
            from isaaclab.devices import Se2Gamepad, Se2GamepadCfg

            self._device = Se2Gamepad(Se2GamepadCfg(sim_device=str(sim_device)))
        elif device_name == "spacemouse":
            from isaaclab.devices import Se2SpaceMouse, Se2SpaceMouseCfg

            self._device = Se2SpaceMouse(Se2SpaceMouseCfg(sim_device=str(sim_device)))

    def update_command_buffer(self, command_buffer: VelocityCommandBuffer) -> None:
        if self._device is None:
            return
        command = tuple(self._device.advance().detach().cpu().tolist())
        is_active = sum(abs(value) for value in command) > 1.0e-3
        if is_active:
            command_buffer.set_vector(0, command)
        elif self._last_active:
            command_buffer.clear(0)
        self._last_active = is_active

    def shutdown(self):
        self._device = None


class _LegacyKeyboardTeleop(TeleopSource):
    KEY_BINDINGS = {
        "W": (1.0, 0.0, 0.0),
        "S": (-1.0, 0.0, 0.0),
        "A": (0.0, 0.6, 0.0),
        "D": (0.0, -0.6, 0.0),
        "Q": (0.0, 0.0, 1.0),
        "E": (0.0, 0.0, -1.0),
    }

    def __init__(self):
        self.active_keys: set[str] = set()
        self.subscription = None
        self._last_active = False

        import carb
        import omni.appwindow

        self._input_interface = carb.input.acquire_input_interface()
        app_window = omni.appwindow.get_default_app_window()
        keyboard = app_window.get_keyboard()
        self.subscription = self._input_interface.subscribe_to_keyboard_events(keyboard, self._keyboard_cb)

    def _keyboard_cb(self, event, *args, **kwargs) -> bool:
        import carb

        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            self.active_keys.add(event.input.name)
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self.active_keys.discard(event.input.name)
        return True

    def update_command_buffer(self, command_buffer: VelocityCommandBuffer) -> None:
        command = [0.0, 0.0, 0.0]
        for key_name, delta in self.KEY_BINDINGS.items():
            if key_name in self.active_keys:
                command[0] += delta[0]
                command[1] += delta[1]
                command[2] += delta[2]

        is_active = sum(abs(value) for value in command) > 1.0e-3
        if is_active:
            command_buffer.set_vector(0, tuple(command))
        elif self._last_active:
            command_buffer.clear(0)
        self._last_active = is_active

    def shutdown(self):
        self.active_keys.clear()
        self.subscription = None


class StabilityCommandGovernor:
    def __init__(self, num_envs: int, device: torch.device | str):
        self.enabled = _bool_env("K1_CMD_GOVERNOR_ENABLED", True)
        self.num_envs = num_envs
        self.device = device
        self.max_linear_x = _float_env("K1_CMD_GOV_MAX_LINEAR_X", 0.06)
        self.max_linear_y = _float_env("K1_CMD_GOV_MAX_LINEAR_Y", 0.0)
        self.max_angular_z = _float_env("K1_CMD_GOV_MAX_ANGULAR_Z", 0.08)
        self.linear_accel = _float_env("K1_CMD_GOV_LINEAR_ACCEL", 0.08)
        self.angular_accel = _float_env("K1_CMD_GOV_ANGULAR_ACCEL", 0.12)
        self.tilt_warn = _float_env("K1_CMD_GOV_TILT_WARN", 0.18)
        self.tilt_stop = _float_env("K1_CMD_GOV_TILT_STOP", 0.24)
        self.base_ang_vel_stop = _float_env("K1_CMD_GOV_BASE_ANG_VEL_STOP", 1.6)
        self.recovery_sec = _float_env("K1_CMD_GOV_RECOVERY_SEC", 1.0)
        self._filtered = torch.zeros((num_envs, 3), dtype=torch.float32, device=device)
        self._recovery_until = torch.zeros(num_envs, dtype=torch.float64)
        self._last_update_time = time.monotonic()
        self._last_log_time = 0.0
        print(
            "[control] K1 stability governor {}: max_linear_x={}, max_linear_y={}, max_angular_z={}, "
            "tilt_warn={}, tilt_stop={}, base_ang_vel_stop={}, recovery_sec={}".format(
                "enabled" if self.enabled else "disabled",
                self.max_linear_x,
                self.max_linear_y,
                self.max_angular_z,
                self.tilt_warn,
                self.tilt_stop,
                self.base_ang_vel_stop,
                self.recovery_sec,
            ),
            flush=True,
        )

    def _extract_policy_obs(self, obs) -> torch.Tensor | None:
        if isinstance(obs, dict):
            obs = obs.get("policy")
        if not isinstance(obs, torch.Tensor):
            return None
        if obs.ndim == 1:
            return obs.unsqueeze(0)
        return obs

    def update(self, command_buffer: VelocityCommandBuffer, obs) -> None:
        if not self.enabled:
            return

        now = time.monotonic()
        dt = max(min(now - self._last_update_time, 0.1), 1.0e-3)
        self._last_update_time = now

        desired = command_buffer.get_desired_commands(self.device)
        desired[:, 0].clamp_(-self.max_linear_x, self.max_linear_x)
        desired[:, 1].clamp_(-self.max_linear_y, self.max_linear_y)
        desired[:, 2].clamp_(-self.max_angular_z, self.max_angular_z)

        policy_obs = self._extract_policy_obs(obs)
        if policy_obs is None or policy_obs.shape[-1] < 9:
            command_buffer.set_filtered_commands(desired)
            self._filtered.copy_(desired)
            return

        projected_gravity = policy_obs[:, 6:9]
        tilt = torch.linalg.vector_norm(projected_gravity[:, :2], dim=-1).detach().cpu()
        base_ang_vel = torch.linalg.vector_norm(policy_obs[:, 3:6], dim=-1).detach().cpu()

        severe_mask = torch.zeros(self.num_envs, dtype=torch.bool)
        recovery_mask = torch.zeros(self.num_envs, dtype=torch.bool)

        for robot_index in range(self.num_envs):
            if tilt[robot_index].item() >= self.tilt_stop or base_ang_vel[robot_index].item() >= self.base_ang_vel_stop:
                desired[robot_index].zero_()
                self._recovery_until[robot_index] = now + self.recovery_sec
                severe_mask[robot_index] = True
                recovery_mask[robot_index] = True
                continue

            if now < self._recovery_until[robot_index]:
                desired[robot_index].zero_()
                recovery_mask[robot_index] = True
                continue

            if tilt[robot_index].item() > self.tilt_warn:
                scale = max(
                    0.0,
                    1.0 - ((tilt[robot_index].item() - self.tilt_warn) / max(self.tilt_stop - self.tilt_warn, 1.0e-6)),
                )
                desired[robot_index] *= scale

        filtered = self._filtered.clone()
        delta = desired - filtered
        max_linear_step = self.linear_accel * dt
        max_angular_step = self.angular_accel * dt
        filtered[:, 0] += torch.clamp(delta[:, 0], -max_linear_step, max_linear_step)
        filtered[:, 1] += torch.clamp(delta[:, 1], -max_linear_step, max_linear_step)
        filtered[:, 2] += torch.clamp(delta[:, 2], -max_angular_step, max_angular_step)

        filtered[recovery_mask] = 0.0
        self._filtered.copy_(filtered)
        command_buffer.set_filtered_commands(filtered)

        if (severe_mask.any() or recovery_mask.any()) and (now - self._last_log_time) >= 1.0:
            self._last_log_time = now
            print(
                "[control] stability governor: tilt={} base_ang_vel={} recovery_active={}".format(
                    [round(value, 3) for value in tilt.tolist()],
                    [round(value, 3) for value in base_ang_vel.tolist()],
                    recovery_mask.tolist(),
                ),
                flush=True,
            )


class StandPoseController:
    def __init__(self, settings: RuntimeSettings, num_envs: int, action_dim: int, device: torch.device | str):
        self.enabled = _bool_env("K1_STAND_POSE_ENABLED", True)
        self.linear_deadband = _float_env("K1_STAND_POSE_LINEAR_DEADBAND", 0.02)
        self.angular_deadband = _float_env("K1_STAND_POSE_ANGULAR_DEADBAND", 0.03)
        self.engage_sec = _float_env("K1_STAND_POSE_ENGAGE_SEC", 0.25)
        self.release_sec = _float_env("K1_STAND_POSE_RELEASE_SEC", 0.60)
        self.reference_frame = _int_env("K1_STAND_POSE_FRAME", 17)
        self.reference_path = os.environ.get("K1_STAND_POSE_NPZ", settings.motion_npz_path)
        self.use_default_pose = _bool_env("K1_STAND_POSE_USE_DEFAULT", False)
        self.num_envs = num_envs
        self.action_dim = action_dim
        self.device = device
        self._blend = torch.ones(num_envs, dtype=torch.float32, device=device)
        self._last_update_time = time.monotonic()
        self._stand_actions = None
        self._action_indices = {joint_name: index for index, joint_name in enumerate(K1_LOCOMOTION_JOINT_NAMES)}

        self.balance_enabled = _bool_env("K1_STAND_BALANCE_ENABLED", False)
        self.gravity_deadband = _float_env("K1_STAND_BALANCE_GRAVITY_DEADBAND", 0.02)
        self.linear_vel_deadband = _float_env("K1_STAND_BALANCE_LINEAR_VEL_DEADBAND", 0.01)
        self.angular_vel_deadband = _float_env("K1_STAND_BALANCE_ANG_VEL_DEADBAND", 0.12)
        self.pitch_gravity_gain = _float_env("K1_STAND_BALANCE_PITCH_GRAVITY_GAIN", -0.10)
        self.pitch_rate_gain = _float_env("K1_STAND_BALANCE_PITCH_RATE_GAIN", -0.025)
        self.pitch_velocity_gain = _float_env("K1_STAND_BALANCE_PITCH_VELOCITY_GAIN", -0.04)
        self.roll_gravity_gain = _float_env("K1_STAND_BALANCE_ROLL_GRAVITY_GAIN", 0.08)
        self.roll_rate_gain = _float_env("K1_STAND_BALANCE_ROLL_RATE_GAIN", 0.020)
        self.roll_velocity_gain = _float_env("K1_STAND_BALANCE_ROLL_VELOCITY_GAIN", 0.030)
        self.hip_pitch_weight = _float_env("K1_STAND_BALANCE_HIP_PITCH_WEIGHT", 0.75)
        self.knee_pitch_weight = _float_env("K1_STAND_BALANCE_KNEE_PITCH_WEIGHT", 0.10)
        self.ankle_pitch_weight = _float_env("K1_STAND_BALANCE_ANKLE_PITCH_WEIGHT", 1.0)
        self.hip_roll_weight = _float_env("K1_STAND_BALANCE_HIP_ROLL_WEIGHT", 0.55)
        self.ankle_roll_weight = _float_env("K1_STAND_BALANCE_ANKLE_ROLL_WEIGHT", 0.85)
        self.max_pitch_delta = _float_env("K1_STAND_BALANCE_MAX_PITCH_DELTA", 0.08)
        self.max_roll_delta = _float_env("K1_STAND_BALANCE_MAX_ROLL_DELTA", 0.06)

        if not self.enabled:
            print("[control] K1 stand pose controller disabled", flush=True)
            return

        try:
            self._stand_actions = self._load_reference_actions()
            print(
                "[control] K1 stand pose controller enabled: frame={} source={} linear_deadband={} "
                "angular_deadband={} engage_sec={} release_sec={} use_default_pose={}".format(
                    self.reference_frame,
                    self.reference_path,
                    self.linear_deadband,
                    self.angular_deadband,
                    self.engage_sec,
                    self.release_sec,
                    self.use_default_pose,
                ),
                flush=True,
            )
            if self.balance_enabled:
                print(
                    "[control] K1 stand balance feedback enabled: pitch(g={}, r={}, v={}) "
                    "roll(g={}, r={}, v={}) max_pitch_delta={} max_roll_delta={}".format(
                        self.pitch_gravity_gain,
                        self.pitch_rate_gain,
                        self.pitch_velocity_gain,
                        self.roll_gravity_gain,
                        self.roll_rate_gain,
                        self.roll_velocity_gain,
                        self.max_pitch_delta,
                        self.max_roll_delta,
                    ),
                    flush=True,
                )
            else:
                print("[control] K1 stand balance feedback disabled", flush=True)
        except Exception as exc:
            self.enabled = False
            print(f"[control] disabling stand pose controller: {exc}", flush=True)

    def _load_reference_actions(self) -> torch.Tensor:
        if self.use_default_pose:
            return torch.zeros((self.num_envs, self.action_dim), dtype=torch.float32, device=self.device)

        pose_path = Path(self.reference_path)
        if not pose_path.is_file():
            raise FileNotFoundError(f"stand pose reference not found: {pose_path}")

        data = np.load(pose_path, allow_pickle=True)
        if "joint_pos" not in data or "joint_names" not in data:
            raise RuntimeError(f"{pose_path} is missing joint_pos or joint_names")

        joint_names = [str(name) for name in data["joint_names"].tolist()]
        joint_positions = data["joint_pos"]
        if joint_positions.ndim != 2:
            raise RuntimeError(f"{pose_path} joint_pos should be rank 2, got shape {joint_positions.shape}")

        frame_index = max(0, min(self.reference_frame, joint_positions.shape[0] - 1))
        joint_name_to_index = {name: index for index, name in enumerate(joint_names)}
        raw_action = torch.zeros((self.action_dim,), dtype=torch.float32, device=self.device)

        for action_index, joint_name in enumerate(K1_LOCOMOTION_JOINT_NAMES):
            if joint_name not in joint_name_to_index:
                raise KeyError(f"stand pose reference is missing joint {joint_name}")
            target_position = float(joint_positions[frame_index, joint_name_to_index[joint_name]])
            offset = _k1_default_joint_offset(joint_name)
            scale = _k1_action_scale_for_joint(joint_name)
            raw_action[action_index] = (target_position - offset) / scale

        return raw_action.unsqueeze(0).repeat(self.num_envs, 1)

    def _add_joint_target_delta(self, actions: torch.Tensor, joint_name: str, delta: torch.Tensor) -> None:
        if joint_name not in self._action_indices:
            raise KeyError(f"Unsupported K1 locomotion joint for stand balance: {joint_name}")
        action_index = self._action_indices[joint_name]
        scale = _k1_action_scale_for_joint(joint_name)
        actions[:, action_index] += delta / scale

    def _balance_stand_actions(self, stand_actions: torch.Tensor, obs) -> torch.Tensor:
        if not self.balance_enabled:
            return stand_actions

        policy_obs = obs.get("policy") if isinstance(obs, dict) else obs
        if not isinstance(policy_obs, torch.Tensor):
            return stand_actions
        if policy_obs.ndim == 1:
            policy_obs = policy_obs.unsqueeze(0)
        if policy_obs.shape[-1] < 9:
            return stand_actions

        base_lin_vel = policy_obs[:, 0:3]
        base_ang_vel = policy_obs[:, 3:6]
        projected_gravity = policy_obs[:, 6:9]

        gravity_x = _soft_deadband(projected_gravity[:, 0], self.gravity_deadband)
        gravity_y = _soft_deadband(projected_gravity[:, 1], self.gravity_deadband)
        base_vx = _soft_deadband(base_lin_vel[:, 0], self.linear_vel_deadband)
        base_vy = _soft_deadband(base_lin_vel[:, 1], self.linear_vel_deadband)
        pitch_rate = _soft_deadband(base_ang_vel[:, 1], self.angular_vel_deadband)
        roll_rate = _soft_deadband(base_ang_vel[:, 0], self.angular_vel_deadband)

        pitch_delta = (
            gravity_x * self.pitch_gravity_gain
            + pitch_rate * self.pitch_rate_gain
            + base_vx * self.pitch_velocity_gain
        ).clamp(-self.max_pitch_delta, self.max_pitch_delta)
        roll_delta = (
            gravity_y * self.roll_gravity_gain
            + roll_rate * self.roll_rate_gain
            + base_vy * self.roll_velocity_gain
        ).clamp(-self.max_roll_delta, self.max_roll_delta)

        balanced = stand_actions.clone()

        for joint_name in ("Left_Hip_Pitch", "Right_Hip_Pitch"):
            self._add_joint_target_delta(balanced, joint_name, -pitch_delta * self.hip_pitch_weight)
        for joint_name in ("Left_Knee_Pitch", "Right_Knee_Pitch"):
            self._add_joint_target_delta(balanced, joint_name, pitch_delta * self.knee_pitch_weight)
        for joint_name in ("Left_Ankle_Pitch", "Right_Ankle_Pitch"):
            self._add_joint_target_delta(balanced, joint_name, pitch_delta * self.ankle_pitch_weight)

        self._add_joint_target_delta(balanced, "Left_Hip_Roll", -roll_delta * self.hip_roll_weight)
        self._add_joint_target_delta(balanced, "Right_Hip_Roll", roll_delta * self.hip_roll_weight)
        self._add_joint_target_delta(balanced, "Left_Ankle_Roll", roll_delta * self.ankle_roll_weight)
        self._add_joint_target_delta(balanced, "Right_Ankle_Roll", -roll_delta * self.ankle_roll_weight)

        return balanced

    def blend(self, policy_actions: torch.Tensor, commanded_actions: torch.Tensor, obs) -> torch.Tensor:
        if not self.enabled or self._stand_actions is None:
            return policy_actions

        now = time.monotonic()
        dt = max(min(now - self._last_update_time, 0.1), 1.0e-3)
        self._last_update_time = now

        motion_requested = (
            torch.linalg.vector_norm(commanded_actions[:, :2], dim=-1) > self.linear_deadband
        ) | (torch.abs(commanded_actions[:, 2]) > self.angular_deadband)

        engage_step = dt / max(self.engage_sec, 1.0e-3)
        release_step = dt / max(self.release_sec, 1.0e-3)
        self._blend = torch.where(
            motion_requested,
            torch.clamp(self._blend - release_step, min=0.0),
            torch.clamp(self._blend + engage_step, max=1.0),
        )

        if torch.all(self._blend <= 1.0e-4):
            return policy_actions

        stand_actions = self._balance_stand_actions(self._stand_actions, obs)
        return torch.lerp(policy_actions, stand_actions, self._blend.unsqueeze(-1))


def create_teleop_source(
    settings: RuntimeSettings,
    sim_device: torch.device | str,
    command_buffer: VelocityCommandBuffer | None = None,
) -> TeleopSource | None:
    if settings.teleop_device == "none":
        return None

    if settings.teleop_device == "cmd_vel":
        if command_buffer is None:
            raise ValueError("cmd_vel teleop requires a shared command buffer.")
        return _CmdVelTeleopSource(settings, command_buffer)

    if settings.teleop_device == "keyboard" and not settings.enable_keyboard_teleop:
        return None

    try:
        return _IsaacLabSe2Device(settings.teleop_device, sim_device)
    except Exception:
        if settings.teleop_device == "keyboard" and settings.enable_keyboard_teleop:
            return _LegacyKeyboardTeleop()
        return None


class ControllerRuntime(ABC):
    @abstractmethod
    def start(self):
        raise NotImplementedError

    @abstractmethod
    def next_action(self, obs) -> torch.Tensor | None:
        raise NotImplementedError

    @abstractmethod
    def shutdown(self):
        raise NotImplementedError


class CmdVelControllerRuntime(ControllerRuntime):
    def __init__(
        self,
        settings: RuntimeSettings,
        command_buffer: VelocityCommandBuffer,
        action_dim: int,
        device: torch.device | str,
        policy=None,
    ):
        self.command_buffer = command_buffer
        self.action_dim = action_dim
        self.device = device
        self.policy = policy
        self.cmd_vel_node = None if settings.teleop_device == "cmd_vel" else CmdVelSubNode(settings, command_buffer)
        self.teleop_source = create_teleop_source(settings, device, command_buffer)
        self.command_governor = StabilityCommandGovernor(command_buffer.num_envs, device)
        self.stand_pose_controller = StandPoseController(settings, command_buffer.num_envs, action_dim, device)

    def start(self):
        if self.cmd_vel_node is not None:
            self.cmd_vel_node.start()

    def next_action(self, obs) -> torch.Tensor:
        if self.cmd_vel_node is not None:
            self.cmd_vel_node.update()
        if self.teleop_source is not None:
            self.teleop_source.update_command_buffer(self.command_buffer)
        self.command_governor.update(self.command_buffer, obs)

        if self.policy is not None:
            policy_obs = obs["policy"] if isinstance(obs, dict) else obs
            commanded_actions = self.command_buffer.get_commands(self.device)
            with torch.inference_mode():
                actions = self.policy(policy_obs)
            return self.stand_pose_controller.blend(actions, commanded_actions, obs)

        return torch.zeros(
            (self.command_buffer.num_envs, self.action_dim),
            dtype=torch.float32,
            device=self.device,
        )

    def shutdown(self):
        if self.teleop_source is not None:
            self.teleop_source.shutdown()
        if self.cmd_vel_node is not None:
            self.cmd_vel_node.shutdown()
