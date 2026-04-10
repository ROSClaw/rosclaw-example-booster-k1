# Booster K1 Isaac Sim Runtime

This directory contains the local Isaac Sim 5.x runtime port for Booster K1 plus the combined launcher for the `external/rosclaw-ros2` submodule.

Run `../scripts/setup_external_environment.sh` from the repo root before using this runtime. That script initializes the required submodules, links the Booster K1 vendor assets into this directory, and applies the local K1 patches to external ROSClaw repos.

## Defaults

- Controller mode defaults to humanoid `cmd_vel`.
- The default world asset is `/Isaac/Environments/Simple_Warehouse/full_warehouse.usd`.
- Stage physics are forced to humanoid-safe defaults before play:
  - Physics Scene present
  - 200 Hz physics
  - GPU dynamics disabled
  - broadphase type `MBP`

## Combined launch

```bash
./isaac-sim-runtime/scripts/launch_k1_isaacsim_rosclaw.sh
```

Useful environment overrides:

- `ISAAC_ENV_SCRIPT`: optional shell script that prepares the Isaac Sim / Isaac Lab Python environment.
- `ISAAC_SIM_PYTHON`: Python executable for the Isaac Sim environment.
- `K1_NAMESPACE`: ROS namespace, default `/k1`.
- `K1_CONTROLLER_BACKEND`: `vendor_cmd_vel` or `gr00t_wbc`.
- `GR00T_WBC_ROOT`: optional local checkout of `GR00T-WholeBodyControl`.
- `K1_SCENE_USD`: override the warehouse world asset.
- `K1_POLICY_CHECKPOINT`: optional locomotion checkpoint.
- `K1_MOTION_FILE`: optional replay asset for `motion_replay`.

## GR00T status

The `gr00t_wbc` backend is wired into the runtime selection surface, but it currently falls back to the vendor K1 locomotion backend unless a K1-specific GR00T adapter is implemented. The public GR00T Python controller in the researched repo is G1-specific today.
