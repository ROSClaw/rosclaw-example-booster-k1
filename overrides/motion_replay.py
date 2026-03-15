from __future__ import annotations

import copy
import os

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg, AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.utils.math import quat_apply, quat_from_euler_xyz, quat_mul, yaw_quat
import torch

from booster_train.assets.robots.booster import BOOSTER_K1_CFG, BOOSTER_K1_ZED_CFG
from booster_train.demo.replay_motion_asset import ReplayMotionAsset
from booster_train.demo.runtime import RuntimeSettings


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


def _build_motion_replay_robot_cfg(use_zed_head: bool) -> ArticulationCfg:
    robot_cfg = copy.deepcopy(BOOSTER_K1_ZED_CFG if use_zed_head else BOOSTER_K1_CFG)
    disable_gravity = _bool_env("K1_SLIDE_DISABLE_GRAVITY", True)
    if robot_cfg.spawn.rigid_props is not None:
        robot_cfg.spawn.rigid_props.disable_gravity = disable_gravity
        robot_cfg.spawn.rigid_props.linear_damping = 0.0
        robot_cfg.spawn.rigid_props.angular_damping = 0.0
    return robot_cfg


@configclass
class MotionReplaySceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())
    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(
            intensity=750.0,
            texture_file=f"{ISAAC_NUCLEUS_DIR}/Materials/Textures/Skies/PolyHaven/kloofendal_43d_clear_puresky_4k.hdr",
        ),
    )
    robot: ArticulationCfg = _build_motion_replay_robot_cfg(use_zed_head=True).replace(prim_path="{ENV_REGEX_NS}/Robot")


class MotionReplaySession:
    def __init__(
        self,
        settings: RuntimeSettings,
        sim: SimulationContext,
        preloaded_motion: ReplayMotionAsset | None = None,
    ):
        self.settings = settings
        self.sim = sim
        self.start_frame = max(0, _int_env("K1_MOTION_START_FRAME", 53))
        self.disable_gravity = _bool_env("K1_SLIDE_DISABLE_GRAVITY", True)

        scene_cfg = MotionReplaySceneCfg(num_envs=settings.env_count, env_spacing=2.5)
        scene_cfg.robot = _build_motion_replay_robot_cfg(settings.use_zed_head).replace(prim_path="{ENV_REGEX_NS}/Robot")
        print(
            "[motion_replay] creating interactive scene (freeze_motion={} start_frame={} disable_gravity={})".format(
                settings.freeze_motion,
                self.start_frame,
                self.disable_gravity,
            ),
            flush=True,
        )
        self.scene = InteractiveScene(scene_cfg)
        print("[motion_replay] interactive scene created", flush=True)
        self.robot: Articulation = self.scene["robot"]
        print("[motion_replay] robot handle resolved", flush=True)

        motion_file = settings.motion_npz_path
        if not os.path.isfile(motion_file):
            raise FileNotFoundError(
                f"Motion file not found: {motion_file}. Run scripts/prepare_k1_assets.py first."
            )

        self._motion_file = motion_file
        self._motion_asset = preloaded_motion or ReplayMotionAsset.load(motion_file, device="cpu")
        self.motion: ReplayMotionAsset | None = None
        self.time_steps = torch.zeros(self.scene.num_envs, dtype=torch.long)
        self.anchor_body_index: int | None = None
        self.root_position_offsets = torch.zeros((self.scene.num_envs, 3), dtype=torch.float32, device=sim.device)
        self.root_yaw_offsets = torch.zeros(self.scene.num_envs, dtype=torch.float32, device=sim.device)

    def reset(self):
        self.sim.reset()
        if self.motion is None:
            print(f"[motion_replay] mapping motion file {self._motion_file}", flush=True)
            self.motion = self._motion_asset.select_tracks(self.robot.body_names, self.robot.joint_names)
            self.anchor_body_index = self.robot.body_names.index("Trunk")
            print("[motion_replay] motion file loaded", flush=True)

        start_frame = min(self.start_frame, self.motion.time_step_total - 1)
        self.time_steps.fill_(start_frame)
        self.root_position_offsets.zero_()
        self.root_yaw_offsets.zero_()
        print(f"[motion_replay] reset to frame {start_frame}", flush=True)

    def _apply_teleop(self, commands: torch.Tensor | None, frame_indices: torch.Tensor):
        if commands is None:
            return None, None, None

        commands = commands.to(self.root_position_offsets.device)
        dt = float(self.sim.get_physics_dt())
        self.root_yaw_offsets += commands[:, 2] * dt

        yaw_offset_quat = quat_from_euler_xyz(
            torch.zeros_like(self.root_yaw_offsets),
            torch.zeros_like(self.root_yaw_offsets),
            self.root_yaw_offsets,
        )
        local_linear_velocity = torch.zeros((self.scene.num_envs, 3), dtype=torch.float32, device=self.sim.device)
        local_linear_velocity[:, 0] = commands[:, 0]
        local_linear_velocity[:, 1] = commands[:, 1]
        motion_heading_quat = yaw_quat(
            self.motion.body_quat_w[frame_indices][:, self.anchor_body_index].to(self.sim.device)
        )
        teleop_heading_quat = quat_mul(yaw_offset_quat, motion_heading_quat)
        linear_velocity_world = quat_apply(teleop_heading_quat, local_linear_velocity)

        self.root_position_offsets += linear_velocity_world * dt
        return commands, linear_velocity_world, yaw_offset_quat

    def step(self, commands: torch.Tensor | None = None):
        if self.motion is None or self.anchor_body_index is None:
            raise RuntimeError("Motion replay session was stepped before reset().")

        frame_indices = self.time_steps
        if not self.settings.freeze_motion:
            self.time_steps += 1
            reset_ids = self.time_steps >= self.motion.time_step_total
            self.time_steps[reset_ids] = 0
            frame_indices = self.time_steps

        commands, linear_velocity_world, yaw_offset_quat = self._apply_teleop(commands, frame_indices)

        root_states = self.robot.data.default_root_state.clone()
        root_states[:, :3] = (
            self.motion.body_pos_w[frame_indices][:, self.anchor_body_index].to(self.sim.device)
            + self.scene.env_origins
            + self.root_position_offsets
        )
        root_states[:, 3:7] = self.motion.body_quat_w[frame_indices][:, self.anchor_body_index].to(self.sim.device)
        if yaw_offset_quat is not None:
            root_states[:, 3:7] = quat_mul(yaw_offset_quat, root_states[:, 3:7])

        if self.settings.freeze_motion:
            root_states[:, 7:10] = 0.0
            root_states[:, 10:] = 0.0
            joint_vel = torch.zeros_like(self.motion.joint_vel[frame_indices].to(self.sim.device))
        else:
            root_states[:, 7:10] = self.motion.body_lin_vel_w[frame_indices][:, self.anchor_body_index].to(self.sim.device)
            root_states[:, 10:] = self.motion.body_ang_vel_w[frame_indices][:, self.anchor_body_index].to(self.sim.device)
            joint_vel = self.motion.joint_vel[frame_indices].to(self.sim.device)

        if linear_velocity_world is not None:
            root_states[:, 7:10] += linear_velocity_world
        if commands is not None:
            root_states[:, 12] += commands[:, 2]

        self.robot.write_root_state_to_sim(root_states)
        self.robot.write_joint_state_to_sim(
            self.motion.joint_pos[frame_indices].to(self.sim.device),
            joint_vel,
        )
        self.scene.write_data_to_sim()
        self.sim.step()
        self.scene.update(self.sim.get_physics_dt())
