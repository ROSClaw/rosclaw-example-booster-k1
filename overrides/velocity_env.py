from __future__ import annotations

import os
import threading

from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm

from booster_train.assets.robots.booster import (
    BOOSTER_K1_CFG,
    BOOSTER_K1_LOCOMOTION_CFG,
    BOOSTER_K1_ZED_CFG,
    K1_LOCOMOTION_JOINT_NAMES,
)
from booster_train.demo.runtime import RuntimeSettings
from booster_train.tasks.manager_based.velocity.mdp import root_height_below_minimum
from booster_train.tasks.manager_based.velocity.robots.k1.rough_env_cfg import K1RoughEnvCfg_PLAY


def _policy_obs_mode() -> str:
    mode = os.environ.get("K1_POLICY_OBS_MODE", "full_body").strip().lower()
    if mode not in {"full_body", "locomotion_only"}:
        raise ValueError(f"Unsupported K1_POLICY_OBS_MODE: {mode}")
    return mode


def _bool_env(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() not in {"0", "false", "no", "off", ""}


def _float_env(name: str, default: float) -> float:
    value = os.environ.get(name)
    if value is None or value == "":
        return default
    return float(value)


def _apply_locomotion_gain_overrides(robot_cfg) -> None:
    if not _bool_env("K1_LOCO_GAIN_OVERRIDE_ENABLED", True):
        return

    leg_stiffness = _float_env("K1_LOCO_LEG_STIFFNESS", 120.0)
    leg_damping = _float_env("K1_LOCO_LEG_DAMPING", 4.0)
    foot_stiffness = _float_env("K1_LOCO_FOOT_STIFFNESS", 50.0)
    foot_damping = _float_env("K1_LOCO_FOOT_DAMPING", 2.5)

    leg_cfg = robot_cfg.actuators["legs"]
    foot_cfg = robot_cfg.actuators["feet"]
    leg_cfg.stiffness = {joint_name: leg_stiffness for joint_name in leg_cfg.joint_names_expr}
    leg_cfg.damping = {joint_name: leg_damping for joint_name in leg_cfg.joint_names_expr}
    foot_cfg.stiffness = foot_stiffness
    foot_cfg.damping = foot_damping

    print(
        "[velocity_env] K1 locomotion gain override enabled: leg_stiffness={} leg_damping={} "
        "foot_stiffness={} foot_damping={}".format(
            leg_stiffness,
            leg_damping,
            foot_stiffness,
            foot_damping,
        ),
        flush=True,
    )


def _apply_fall_reset_override(env_cfg: K1RoughEnvCfg_PLAY) -> None:
    if not _bool_env("K1_FALL_RESET_ENABLED", True):
        return

    minimum_height = _float_env("K1_FALL_RESET_MIN_HEIGHT", 0.38)
    env_cfg.terminations.crouch_or_fall = DoneTerm(
        func=root_height_below_minimum,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "minimum_height": minimum_height,
        },
    )
    print(f"[velocity_env] K1 fall reset enabled: minimum_height={minimum_height}", flush=True)


def _apply_reset_pose_overrides(env_cfg: K1RoughEnvCfg_PLAY) -> None:
    env_cfg.events.reset_base.params = {
        "pose_range": {"x": (0.0, 0.0), "y": (0.0, 0.0), "yaw": (0.0, 0.0)},
        "velocity_range": {
            "x": (0.0, 0.0),
            "y": (0.0, 0.0),
            "z": (0.0, 0.0),
            "roll": (0.0, 0.0),
            "pitch": (0.0, 0.0),
            "yaw": (0.0, 0.0),
        },
    }
    env_cfg.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
    env_cfg.events.reset_robot_joints.params["velocity_range"] = (0.0, 0.0)
    print("[velocity_env] K1 reset pose override enabled: deterministic base and joint resets", flush=True)


def build_high_level_env_cfg(settings: RuntimeSettings) -> K1RoughEnvCfg_PLAY:
    env_cfg = K1RoughEnvCfg_PLAY()
    env_cfg.scene.num_envs = settings.env_count

    if settings.terrain == "flat":
        env_cfg.scene.terrain.terrain_type = "plane"
        env_cfg.scene.terrain.terrain_generator = None
        env_cfg.curriculum.terrain_levels = None

    obs_mode = _policy_obs_mode()
    if obs_mode == "locomotion_only":
        robot_cfg = BOOSTER_K1_LOCOMOTION_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        _apply_locomotion_gain_overrides(robot_cfg)
        env_cfg.scene.robot = robot_cfg
        locomotion_joint_cfg = SceneEntityCfg(
            "robot",
            joint_names=K1_LOCOMOTION_JOINT_NAMES,
            preserve_order=True,
        )
        env_cfg.observations.policy.joint_pos.params = {"asset_cfg": locomotion_joint_cfg}
        env_cfg.observations.policy.joint_vel.params = {"asset_cfg": locomotion_joint_cfg}
    else:
        robot_cfg = BOOSTER_K1_ZED_CFG if settings.use_zed_head else BOOSTER_K1_CFG
        env_cfg.scene.robot = robot_cfg.replace(prim_path="{ENV_REGEX_NS}/Robot")

    _apply_fall_reset_override(env_cfg)
    _apply_reset_pose_overrides(env_cfg)
    env_cfg.configure_for_num_envs()
    return env_cfg


class IsaacSimK1VelocityEnvWrapper:
    def __init__(self, env: ManagerBasedRLEnv, command_buffer):
        self._env = env
        self._env.external_command_buffer = command_buffer
        self.command_buffer = command_buffer
        self.device = self._env.device
        self.num_envs = self._env.num_envs
        self.lock = threading.Lock()

    def reset(self):
        obs, _ = self._env.reset()
        return obs

    def step(self, action):
        obs, _, _, _, _ = self._env.step(action)
        return obs

    def close(self):
        return self._env.close()

    @property
    def robot(self):
        return self._env.scene["robot"]

    @property
    def action_dim(self) -> int:
        return len(K1_LOCOMOTION_JOINT_NAMES)

    @property
    def dt(self) -> float:
        return self._env.step_dt
