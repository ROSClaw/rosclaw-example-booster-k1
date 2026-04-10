# ROSClaw Booster K1 Example

This repo contains the Booster K1 example wiring for ROSClaw/OpenClaw and the
Isaac Sim 5.x helper runtime. External code is referenced as git submodules;
local modifications for those repos are kept as patch files under `patches/`
and are applied by the setup script.

## External Setup

After cloning, initialize the external repos and apply the local K1 integration
patches:

```bash
./scripts/setup_external_environment.sh
```

If this checkout already has copied vendor directories from earlier local
experiments, replace them with submodule-backed symlinks:

```bash
./scripts/setup_external_environment.sh --force-vendor-links
```

The setup script does the following:

- initializes the Booster K1 RL/assets, ROSClaw ROS 2, ROSClaw plugin, and
  Booster ROS 2 SDK submodules
- links `isaac-sim-runtime/vendor/booster_assets` and
  `isaac-sim-runtime/vendor/booster_train` to the Booster K1 RL submodule
- applies `patches/booster-k1-rl-runtime-overrides.patch` to the Booster K1 RL
  submodule
- applies `patches/rosclaw-ros2-k1-bringup.patch` to the ROSClaw ROS 2 submodule
- applies `patches/rosclaw-plugin-k1-openclaw.patch` to the ROSClaw plugin
  submodule
- syncs the patched ROSClaw plugin into
  `~/.openclaw/extensions/rosclaw` when that default OpenClaw install exists

Those submodule working-tree modifications are intentionally local setup state.
Do not commit dirty submodule changes; commit changes to the patch files here
instead.

Use the setup script instead of a blanket recursive submodule update. The
upstream Booster K1 RL repo currently contains a nested `booster_assets`
submodule URL that points at a local absolute path; this repo tracks
`https://github.com/BoosterRobotics/booster_assets` as a top-level submodule to
make fresh checkouts portable.

## OpenClaw Profile

A default OpenClaw install needs the K1 ROSClaw transport profile before it can
drive this simulator:

```bash
./simulators/isaac-sim/scripts/configure_openclaw_k1.sh
```

That script points the ROSClaw plugin at `ws://127.0.0.1:9090`, sets the robot
name to `Booster K1`, sets the namespace to `/k1`, installs conservative K1
safety limits, and leaves unrelated OpenClaw auth/provider settings alone.

To undo the K1 profile and return OpenClaw to a non-K1 configuration:

```bash
./simulators/isaac-sim/scripts/unconfigure_openclaw_k1.sh
```

## Isaac Sim

Start the verified WebRTC simulator stack:

```bash
./simulators/isaac-sim/scripts/run_k1_isaac_sim.sh --mode webrtc
```

Start with a local Isaac Sim GUI:

```bash
xhost +local:root
./simulators/isaac-sim/scripts/run_k1_isaac_sim.sh --mode gui
```

Stop containers started by the helper:

```bash
./simulators/isaac-sim/scripts/stop_k1_isaac_sim.sh
```

The default simulator mode is `K1_CONTROLLER_MODE=kinematic_gait`. It keeps the
full-body K1 upright, responds to streamed `/k1/cmd_vel`, and publishes odom.
Dynamic policy mode remains available for checkpoint diagnostics but is not the
verified default.

## What Not To Commit

The following are local setup/build artifacts and are ignored:

- `isaac-sim-runtime/vendor/booster_assets`
- `isaac-sim-runtime/vendor/booster_train`
- `isaac-sim-runtime/logs`
- `real-hardware/build`
- `real-hardware/install`
- `real-hardware/log`
- local `.pt`, `.pth`, and `.onnx` checkpoint experiments
