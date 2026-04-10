# Booster K1 Isaac Sim / OpenClaw

This directory contains local helper scripts for the Booster K1 Isaac Sim example and the OpenClaw K1 profile.

## Do I Need To Change Anything For A Default OpenClaw Install?

Yes.

If you install OpenClaw with its default settings and want it to target this K1 stack, you still need to apply the K1-specific robot transport settings:

- point the `rosclaw` plugin at `ws://127.0.0.1:9090`
- set the robot name to `Booster K1`
- set the robot namespace to `/k1`
- set the K1 safety limits used by this simulator profile
- add a longer gateway handshake timeout for this machine's OpenClaw CLI path
- leave `rosclaw-autonomy` unconfigured unless you also install its message package support into the simulator runtime

The helper scripts here do that for you.

What the scripts do not touch:

- your LLM provider auth
- your Discord token
- your gateway token
- your general OpenClaw profile outside the K1-specific `rosclaw` config block

The only runtime patch they make outside `~/.openclaw/openclaw.json` is a small local patch to the installed OpenClaw gateway bundle so `OPENCLAW_HANDSHAKE_TIMEOUT_MS` works in normal runtime mode. That local patch may need to be re-applied after an OpenClaw update.

## Scripts

- `./scripts/configure_openclaw_k1.sh`
  - applies the K1 OpenClaw config
  - patches the installed OpenClaw gateway runtime to accept `OPENCLAW_HANDSHAKE_TIMEOUT_MS`
  - writes a user-systemd drop-in with `OPENCLAW_HANDSHAKE_TIMEOUT_MS=10000`
  - leaves `rosclaw-autonomy` absent by default for the K1 Isaac Sim profile
  - reloads and restarts `openclaw-gateway.service`

- `./scripts/unconfigure_openclaw_k1.sh`
  - removes the K1-specific `rosclaw` config payload and any K1 autonomy payload if present
  - removes the K1 handshake timeout drop-in
  - removes the local gateway timeout patch
  - reloads and restarts `openclaw-gateway.service`
  - leaves your general OpenClaw auth and channel credentials alone

- `./scripts/run_k1_isaac_sim.sh --mode webrtc`
  - runs the K1 Isaac Sim container with the local mounted K1 entrypoint override
  - starts the warehouse scene and ROSClaw sidecar
  - enables Isaac Sim WebRTC streaming
  - defaults to `K1_CONTROLLER_MODE=kinematic_gait`, which keeps the K1 upright and walks from `/k1/cmd_vel` while publishing `/k1/odom`
  - auto-mounts `./k1_warehouse_demo_override.py`, `./docker/entrypoint-k1.sh`, and the local `isaac-sim-runtime` tree so the bundled image uses the corrected K1 warehouse bootstrap

- `./scripts/run_k1_isaac_sim.sh --mode gui`
  - runs the K1 Isaac Sim runtime in GUI mode
  - bypasses the image entrypoint because the bundled entrypoint always forces `--headless`
  - also starts `rosclaw_bringup` in the same container

- `./scripts/stop_k1_isaac_sim.sh`
  - stops any local K1 Isaac Sim containers created by these helpers

## Usage

Configure OpenClaw for K1:

```bash
./scripts/configure_openclaw_k1.sh
```

Undo the K1-specific OpenClaw changes:

```bash
./scripts/unconfigure_openclaw_k1.sh
```

Run the simulator with WebRTC:

```bash
./scripts/run_k1_isaac_sim.sh --mode webrtc
```

Run the simulator with the Isaac Sim GUI:

```bash
xhost +local:root
./scripts/run_k1_isaac_sim.sh --mode gui
```

Stop any K1 simulator container started by the helper:

```bash
./scripts/stop_k1_isaac_sim.sh
```

## Controller Modes

The default controller is `K1_CONTROLLER_MODE=kinematic_gait`. It uses the full-body K1 asset, holds the root upright, animates an alternating walking gait, and integrates `/k1/cmd_vel` into `/k1/odom`. This is the verified mode for OpenClaw smoke tests: a sustained `0.5 m/s` forward command for 7 seconds produced about `3.45 m` of odom displacement while keeping trunk height at about `0.57 m`. Reverse movement is enabled by default with `K1_KINEMATIC_LIN_X_MIN=-0.8`; policy mode still uses the policy's trained forward-only range.

`K1_CONTROLLER_MODE=policy` keeps the Isaac Lab policy path available for diagnostics. It can load TorchScript exports or RSL-RL checkpoints, but the current tested K1 checkpoint at `/home/marnett5/Developer/Isaac-Sim-Go2/Booster-K1-RL-Isaac-Lab-Walking-Policy-Trainer/checkpoints/k1_cmd_vel_latest.pt` moves briefly and then falls in canonical Isaac Lab play, so it is not the default. Use policy mode only when validating a new dynamic walking checkpoint.

For visible movement, publish `/k1/cmd_vel` continuously. The controller intentionally clears stale commands after about `0.5 s`, so a single one-shot publish is treated as a short pulse.

OpenClaw movement requests should use the RosClaw plugin `ros2_cmd_vel` tool rather than a one-shot `ros2_publish` Twist. The installed local RosClaw plugin has that tool plus prompt guidance for `/k1/cmd_vel`; a K1 forward smoke test moved `0.630 m` (`2.07 ft`) and a reverse smoke test moved `0.325 m` (`1.07 ft`) with odom height stable at `0.570 m`.

## Useful Overrides

These helpers accept environment overrides when you need to point at a different local layout.

OpenClaw configuration helpers:

- `OPENCLAW_CONFIG_FILE`
- `OPENCLAW_DIST_DIR`
- `ROSCLAW_AUTONOMY_PLUGIN_PATH`
- `ROSBRIDGE_URL`
- `ROBOT_NAME`
- `ROBOT_NAMESPACE`
- `OPENCLAW_HANDSHAKE_TIMEOUT_MS`

Isaac Sim runner:

- `IMAGE`
- `ROS_DOMAIN_ID`
- `PUBLIC_IP`
- `K1_NAMESPACE`
- `K1_PORTED_RUNTIME_HOST_ROOT`
- `K1_PORTED_RUNTIME_CONTAINER_ROOT`
- `K1_POLICY_HOST_PATH`
- `K1_POLICY_CONTAINER_PATH`
- `K1_SCENE_USD`
- `K1_CONTROL_PERIOD_MS`
- `K1_POLICY_DECIMATION`
- `K1_POLICY_OBS_DIM`
- `K1_POLICY_HEIGHT_SCAN_DIM`
- `K1_ROBOT_Z`
- `K1_ROBOT_ASSET_MODE`
- `K1_CONTROLLER_MODE`
- `K1_KINEMATIC_DT_MODE`
- `K1_KINEMATIC_MAX_DT`
- `K1_KINEMATIC_SPEED_SCALE`
- `K1_KINEMATIC_LIN_X_MIN`
- `K1_KINEMATIC_LIN_X_MAX`
- `K1_VIEW_CAMERA_MODE`
- `K1_VIEW_CAMERA_EYE`
- `K1_VIEW_CAMERA_TARGET`
- `K1_VIEW_CAMERA_FOCAL_LENGTH`
- `K1_APP_OVERRIDE`
- `K1_BOOSTER_ASSET_OVERRIDE`

## Notes

- The helper runner sets `/Isaac/Environments/Simple_Warehouse/full_warehouse.usd` by default. Override `K1_SCENE_USD` only when you want a different stage.
- The helper runner also mounts `./k1_warehouse_demo_override.py`, `./docker/entrypoint-k1.sh`, and the local `isaac-sim-runtime` tree by default. That override boots the warehouse stage with a 200 Hz physics scene, GPU dynamics off, `MBP` broadphase, the full-body K1 asset, an observer camera, and the stable `kinematic_gait` controller.
- The default observer camera is controlled by `K1_VIEW_CAMERA_EYE`, `K1_VIEW_CAMERA_TARGET`, and `K1_VIEW_CAMERA_FOCAL_LENGTH`. Set `K1_VIEW_CAMERA_MODE=front_cam` if you want the robot-mounted camera as the active WebRTC viewport.
- `K1_BOOSTER_ASSET_OVERRIDE` is left unset by default. Use the asset override only for targeted debugging.
- `./scripts/configure_openclaw_k1.sh` leaves `rosclaw-autonomy` absent unless you explicitly set `ENABLE_ROSCLAW_AUTONOMY_PLUGIN=true`. The sim container does not ship `rosclaw_autonomy_msgs`, so enabling it by default causes rosbridge noise and failed subscriptions.
- If you still see `rosclaw_autonomy_msgs` import errors in rosbridge logs, your current OpenClaw profile still has the autonomy plugin enabled. Re-run `./scripts/configure_openclaw_k1.sh` to put the K1 profile back on the supported config.
- GUI mode requires a local X11 session. The helper mounts `/tmp/.X11-unix` and uses your current `DISPLAY`.
