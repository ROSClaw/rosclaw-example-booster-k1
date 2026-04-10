#!/usr/bin/env python3

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
import threading
import time
import traceback


SCRIPT_DIR = Path(__file__).resolve().parent
RUNTIME_ROOT = SCRIPT_DIR.parent
BOOSTER_ASSETS_SOURCE = RUNTIME_ROOT / "vendor" / "booster_assets" / "src"
BOOSTER_TRAIN_SOURCE = RUNTIME_ROOT / "vendor" / "booster_train" / "source" / "booster_train"

for path in (BOOSTER_ASSETS_SOURCE, BOOSTER_TRAIN_SOURCE):
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)

from isaaclab.app import AppLauncher

from booster_train.demo.runtime import add_runtime_args, runtime_settings_from_args
from booster_train.demo.replay_motion_asset import ReplayMotionAsset


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the locally ported Booster K1 Isaac Sim runtime."
    )
    add_runtime_args(parser)
    parser.add_argument(
        "--controller-backend",
        type=str,
        default=os.environ.get("K1_CONTROLLER_BACKEND", "vendor_cmd_vel"),
        choices=("vendor_cmd_vel", "gr00t_wbc"),
        help="Controller backend for cmd_vel mode.",
    )
    parser.add_argument(
        "--gr00t-root",
        type=str,
        default=os.environ.get("GR00T_WBC_ROOT", ""),
        help="Optional filesystem path to a local GR00T-WholeBodyControl checkout.",
    )
    parser.add_argument(
        "--gr00t-strict",
        action="store_true",
        help="Fail fast instead of falling back when the GR00T backend is unavailable for K1.",
    )
    AppLauncher.add_app_launcher_args(parser)
    return parser.parse_args()


def main():
    args = parse_args()
    settings = runtime_settings_from_args(args)
    preloaded_motion = None
    if settings.control_mode == "motion_replay":
        print(f"[runtime] preloading motion asset {settings.motion_npz_path}", flush=True)
        preloaded_motion = ReplayMotionAsset.load(settings.motion_npz_path, device="cpu")
        print("[runtime] motion asset preloaded", flush=True)

    args.enable_cameras = True
    required_kit_args = (
        "--enable isaacsim.ros2.bridge",
        "--enable isaacsim.storage.native",
    )
    existing_kit_args = getattr(args, "kit_args", "") or ""
    missing_kit_args = [kit_arg for kit_arg in required_kit_args if kit_arg not in existing_kit_args]
    if missing_kit_args:
        args.kit_args = f"{existing_kit_args} {' '.join(missing_kit_args)}".strip()

    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    import isaaclab.sim as sim_utils
    import omni.timeline
    from isaaclab.scene import InteractiveScene
    from isaaclab.sim import SimulationContext

    from booster_train.assets.robots.booster import BOOSTER_K1_CFG, BOOSTER_K1_ZED_CFG
    from booster_train.demo.camera import add_front_camera, create_front_cam_omnigraph
    from booster_train.demo.control import CmdVelControllerRuntime, VelocityCommandBuffer, create_teleop_source
    from booster_train.demo.gr00t_backend import create_gr00t_controller_runtime
    from booster_train.demo.image_transport import K1CompressedCameraRelay
    from booster_train.demo.motion_replay import MotionReplaySceneCfg, MotionReplaySession
    from booster_train.demo.scene_bootstrap import bootstrap_stage
    from booster_train.demo.scene_props import spawn_scene_prop_assets
    from booster_train.demo.sdk_compat import BoosterSdkCompatShim

    timeline = omni.timeline.get_timeline_interface()

    def setup_runtime_scene():
        spawn_scene_prop_assets(settings)
        for robot_index in settings.iter_robot_indices():
            add_front_camera(settings, robot_index)
            create_front_cam_omnigraph(settings, robot_index)

    def wait_for_app_running(timeout_sec: float = 30.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if simulation_app.is_running():
                return True
            simulation_app.update()
        return simulation_app.is_running()

    def create_ros_nodes():
        try:
            import rclpy
            from booster_train.demo.ros_bridge import K1RosStatePublisher
        except Exception as exc:
            print(f"[warn] ROS publishing disabled inside Isaac Sim runtime: {exc}", flush=True)
            return None, None, None

        if not rclpy.ok():
            rclpy.init(args=None)
        state_publisher = K1RosStatePublisher(settings)
        camera_relay = K1CompressedCameraRelay(settings)
        camera_relay.start()
        return state_publisher, camera_relay, rclpy

    def load_policy(env):
        if not settings.policy_checkpoint:
            return None
        from rsl_rl.runners import OnPolicyRunner
        from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
        from booster_train.tasks.manager_based.velocity.robots.k1.ppo_cfg import PPORunnerCfg

        runner_cfg = PPORunnerCfg()
        vec_env = RslRlVecEnvWrapper(env, clip_actions=runner_cfg.clip_actions)
        runner = OnPolicyRunner(vec_env, runner_cfg.to_dict(), log_dir=None, device=env.device)
        runner.load(settings.policy_checkpoint)
        return runner.get_inference_policy(device=env.device)

    def create_interactive_scene(sim: SimulationContext):
        scene_cfg = MotionReplaySceneCfg(num_envs=settings.env_count, env_spacing=2.5)
        robot_cfg = BOOSTER_K1_ZED_CFG if settings.use_zed_head else BOOSTER_K1_CFG
        scene_cfg.robot = robot_cfg.replace(prim_path="{ENV_REGEX_NS}/Robot")
        scene = InteractiveScene(scene_cfg)
        return scene, scene["robot"]

    def build_low_state_msg(robot):
        sdk = sdk_shim.sdk if sdk_shim is not None else None
        if sdk is None or not hasattr(sdk, "LowState"):
            return None
        low_state = sdk.LowState()
        joint_count = min(len(robot.joint_names), len(low_state.motor_state))
        for idx in range(joint_count):
            low_state.motor_state[idx].q = robot.data.joint_pos[0, idx].item()
            low_state.motor_state[idx].dq = robot.data.joint_vel[0, idx].item()
        low_state.imu_state.quaternion[0] = robot.data.root_quat_w[0, 0].item()
        low_state.imu_state.quaternion[1] = robot.data.root_quat_w[0, 1].item()
        low_state.imu_state.quaternion[2] = robot.data.root_quat_w[0, 2].item()
        low_state.imu_state.quaternion[3] = robot.data.root_quat_w[0, 3].item()
        low_state.imu_state.gyroscope[0] = robot.data.root_ang_vel_b[0, 0].item()
        low_state.imu_state.gyroscope[1] = robot.data.root_ang_vel_b[0, 1].item()
        low_state.imu_state.gyroscope[2] = robot.data.root_ang_vel_b[0, 2].item()
        return low_state

    class LowCmdController:
        def __init__(self, robot):
            self.robot = robot
            self.lock = threading.Lock()
            self.latest_msg = None

        def handle_lowcmd(self, msg):
            with self.lock:
                self.latest_msg = msg

        def step(self):
            with self.lock:
                msg = self.latest_msg
            if msg is None:
                return

            joint_pos = self.robot.data.joint_pos.clone()
            joint_vel = self.robot.data.joint_vel.clone()
            joint_count = min(joint_pos.shape[1], len(msg.motor_cmd))
            for idx in range(joint_count):
                joint_pos[0, idx] = msg.motor_cmd[idx].q
                joint_vel[0, idx] = getattr(msg.motor_cmd[idx], "dq", 0.0)
            self.robot.write_joint_state_to_sim(joint_pos, joint_vel)

    ros_state_publisher, camera_relay, rclpy_module = create_ros_nodes()
    sdk_shim = None
    motion_replay_teleop = None
    controller = None

    try:
        if settings.control_mode == "motion_replay":
            sim = SimulationContext(
                sim_utils.SimulationCfg(
                    dt=settings.sim_dt,
                    render_interval=settings.render_interval,
                )
            )
            bootstrap_stage(settings)
            command_buffer = VelocityCommandBuffer(settings.env_count, sim.device)
            session = MotionReplaySession(settings, sim, preloaded_motion=preloaded_motion)
            session.reset()
            setup_runtime_scene()
            motion_replay_teleop = create_teleop_source(settings, sim.device, command_buffer)
            if settings.sdk_compat:
                sdk_shim = BoosterSdkCompatShim(command_buffer)
                sdk_shim.start()

            while True:
                if motion_replay_teleop is not None:
                    motion_replay_teleop.update_command_buffer(command_buffer)
                session.step(command_buffer.get_commands(sim.device))
                sim_time_sec = timeline.get_current_time()
                if ros_state_publisher is not None:
                    ros_state_publisher.publish(session.robot, sim_time_sec)
                if sdk_shim is not None:
                    low_state = build_low_state_msg(session.robot)
                    if low_state is not None:
                        sdk_shim.publish_low_state(low_state)

        elif settings.control_mode == "cmd_vel":
            from isaaclab.envs import ManagerBasedRLEnv
            from booster_train.demo.velocity_env import IsaacSimK1VelocityEnvWrapper, build_high_level_env_cfg

            env_cfg = build_high_level_env_cfg(settings)
            env = ManagerBasedRLEnv(env_cfg)
            bootstrap_stage(settings)
            setup_runtime_scene()
            if not wait_for_app_running():
                raise RuntimeError("Isaac Sim never entered a running state after cmd_vel setup.")

            command_buffer = VelocityCommandBuffer(settings.env_count, env.device)
            wrapper = IsaacSimK1VelocityEnvWrapper(env, command_buffer)
            policy = load_policy(env)
            if args.controller_backend == "gr00t_wbc":
                controller = create_gr00t_controller_runtime(
                    settings,
                    command_buffer,
                    wrapper.action_dim,
                    env.device,
                    fallback_policy=policy,
                    gr00t_root=args.gr00t_root,
                    strict=args.gr00t_strict,
                )
            else:
                controller = CmdVelControllerRuntime(
                    settings,
                    command_buffer,
                    wrapper.action_dim,
                    env.device,
                    policy=policy,
                )

            print("[runtime] starting humanoid controller before the first simulation step", flush=True)
            controller.start()
            if settings.sdk_compat:
                sdk_shim = BoosterSdkCompatShim(command_buffer)
                sdk_shim.start()

            obs = wrapper.reset()
            while True:
                actions = controller.next_action(obs)
                obs = wrapper.step(actions)
                sim_time_sec = timeline.get_current_time()
                if ros_state_publisher is not None:
                    ros_state_publisher.publish(wrapper.robot, sim_time_sec)
                if sdk_shim is not None:
                    low_state = build_low_state_msg(wrapper.robot)
                    if low_state is not None:
                        sdk_shim.publish_low_state(low_state)

        else:
            sim = SimulationContext(
                sim_utils.SimulationCfg(
                    dt=settings.sim_dt,
                    render_interval=settings.render_interval,
                )
            )
            scene, robot = create_interactive_scene(sim)
            bootstrap_stage(settings)
            sim.reset()
            setup_runtime_scene()
            if not wait_for_app_running():
                raise RuntimeError("Isaac Sim never entered a running state after sdk_lowcmd setup.")
            lowcmd_controller = LowCmdController(robot)
            command_buffer = VelocityCommandBuffer(settings.env_count)
            sdk_shim = BoosterSdkCompatShim(command_buffer, lowcmd_handler=lowcmd_controller.handle_lowcmd)
            sdk_shim.start()

            while True:
                lowcmd_controller.step()
                scene.write_data_to_sim()
                sim.step()
                scene.update(sim.get_physics_dt())
                sim_time_sec = timeline.get_current_time()
                if ros_state_publisher is not None:
                    ros_state_publisher.publish(robot, sim_time_sec)
                low_state = build_low_state_msg(robot)
                if low_state is not None:
                    sdk_shim.publish_low_state(low_state)

    except BaseException as exc:
        print(f"[runtime] unhandled exception: {type(exc).__name__}: {exc}", flush=True)
        traceback.print_exc()
        raise
    finally:
        if controller is not None:
            controller.shutdown()
        if motion_replay_teleop is not None:
            motion_replay_teleop.shutdown()
        if sdk_shim is not None:
            sdk_shim.shutdown()
        if camera_relay is not None:
            camera_relay.shutdown()
            camera_relay.destroy_node()
        if ros_state_publisher is not None:
            ros_state_publisher.destroy_node()
        if rclpy_module is not None and rclpy_module.ok():
            rclpy_module.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
