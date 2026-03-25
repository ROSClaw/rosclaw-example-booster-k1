# ROSClaw Booster K1

This repo contains the ROSClaw Booster K1 hardware workspace and the DDS setup that was used to reach the robot from the host over ROS 2 Humble.

## Layout

- `dds-profile.xml`: project-root Fast DDS profile aligned with the robot and this host
- `setup-dds-env.sh`: shell setup that removes Conda from the active environment and exports the Fast DDS profile
- `real-hardware/`: ROS 2 workspace used for the host-side SDK, DDS tests, and relay package
- `real-hardware/src/booster_robotics_sdk_ros2`: upstream Booster ROS 2 SDK as a git submodule
- `real-hardware/src/k1_low_level_relay`: robot-local relay package for republishing bare DDS low-level topics as ROS 2 topics under `/k1`

## Setup

Clone with submodules, or initialize the SDK submodule after cloning:

```bash
git submodule update --init --recursive
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

## Current State

- Host-side discovery works for the main Booster RPC services.
- Safe RPC calls to the robot were verified from the host.
- Bare DDS state topics such as `/low_state` did not deliver reliably to the host directly.
- The `k1_low_level_relay` package worked as a robot-local workaround and republished state topics that the host could read as `/k1/low_state` and `/k1/joint_states`.

The detailed investigation notes and test results are in `real-hardware/DDS_HOST_MATCH_REPORT.md`.
