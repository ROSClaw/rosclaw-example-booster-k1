# ROSClaw Booster K1

This repo contains both Booster K1 paths used by ROSClaw:

- a real-hardware ROS 2 workspace and DDS setup for the physical Booster K1
- an Isaac Sim 5.x/OpenClaw helper runtime for the simulated Booster K1

External code is referenced as git submodules. Local modifications for those
external repos are kept as patch files under `patches/` and are applied by the
setup script.

## Layout

- `dds-profile.xml`: project-root Fast DDS profile aligned with the robot and this host
- `setup-dds-env.sh`: shell setup that removes Conda from the active environment and exports the Fast DDS profile
- `real-hardware/`: ROS 2 workspace used for the host-side SDK, DDS tests, and relay package
- `real-hardware/src/booster_robotics_sdk_ros2`: upstream Booster ROS 2 SDK as a git submodule
- `real-hardware/src/rosclaw-ros2-autonomy`: ROSClaw autonomy overlay as a git submodule
- `real-hardware/src/k1_low_level_relay`: robot-local relay package for republishing bare DDS low-level topics as ROS 2 topics under `/k1`
- `simulators/isaac-sim/`: Docker/WebRTC helper scripts for the K1 Isaac Sim runtime
- `isaac-sim-runtime/`: local Isaac Sim 5.x runtime entrypoint used by the simulator helper
- `external/`: submodules used by the simulator setup script
- `patches/`: reproducible local modifications applied to external submodules

## Real Hardware DDS Setup

Initialize the Booster SDK submodule if needed:

```bash
git submodule update --init -- \
  real-hardware/src/booster_robotics_sdk_ros2 \
  real-hardware/src/rosclaw-ros2-autonomy
```

Source the DDS environment from the repo root:

```bash
source ./setup-dds-env.sh
```

That script:

- removes Conda Python paths and environment variables
- sources ROS 2 Humble and the `real-hardware` overlay if it has been built
- selects `rmw_fastrtps_cpp`
- points Fast DDS at the project-root `dds-profile.xml`

Current real-hardware state:

- Host-side discovery works for the main Booster RPC services.
- Safe RPC calls to the robot were verified from the host.
- Bare DDS state topics such as `/low_state` did not deliver reliably to the host directly.
- The `k1_low_level_relay` package worked as a robot-local workaround and republished state topics that the host could read as `/k1/low_state` and `/k1/joint_states`.

The detailed investigation notes and test results are in
`real-hardware/DDS_HOST_MATCH_REPORT.md`.

Launch the host-side K1 stack with:

```bash
./real-hardware/bringup_openclaw_k1.sh
```

That wrapper now defaults to the `k1` ROSClaw platform config instead of the
generic profile and, when `rosclaw_autonomy` is built in the local
`real-hardware` overlay, starts `rosclaw_autonomy` a few seconds after
`rosclaw_bringup`.

Build the local real-hardware overlay with autonomy on top of the existing
`rosclaw-ros2` install:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd real-hardware
colcon build --packages-select rosclaw_autonomy_msgs rosclaw_autonomy
```

## ROS 2 CLI Caveat

If `/rosclaw/manifest` shows the full graph but `ros2 topic list` only shows a
small subset, the robot is usually fine and the local ROS 2 daemon is stale or
was started from a different DDS environment.

After sourcing `./setup-dds-env.sh`, either:

```bash
ros2 daemon stop
ros2 topic list
```

or query the graph directly:

```bash
ros2 topic list --no-daemon
ros2 service list --no-daemon
ros2 node list --no-daemon
```

## Isaac Sim External Setup

After cloning, initialize the simulator external repos and apply the local K1
integration patches:

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
- `real-hardware/ws`
- `real-hardware/build`
- `real-hardware/install`
- `real-hardware/log`
- generated real-hardware env/XML/log files
- local `.pt`, `.pth`, and `.onnx` checkpoint experiments
