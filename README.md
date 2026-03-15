# ROSClaw Booster K1 Isaac Example

This example packages a single-container Booster K1 Isaac Sim demo that ROSClaw
can drive through the local `rosclaw-plugin` checkout.

The default runtime is now a gravity-free slide mode, not the walking-policy
path. It uses the existing Booster motion-replay stack in frozen mode, holds
the robot on a fixed K1 pose, and applies `/k1/cmd_vel` by sliding the robot
base through the scene. That keeps ROSClaw integration, rosbridge, odometry,
camera topics, and `/k1/cmd_vel` working without depending on policy balance or
sim2real locomotion behavior.

## Prerequisites

- Linux host with NVIDIA GPU, Docker Engine, Docker Compose plugin, and NVIDIA
  Container Toolkit installed.
- Sibling checkouts at `../rosclaw-plugin` and `../rosclaw-ros2`.
- Sibling K1 runtime sources at:
  - `../../Isaac-Sim-Go2/ROSClaw-Vision`
  - `../../Isaac-Sim-Go2/Booster-K1/booster_train`
  - `../../Isaac-Sim-Go2/Booster-K1/booster_assets`
- NVIDIA Isaac Sim WebRTC Streaming Client installed on the machine that will
  view the simulation.
- OpenClaw Gateway already installed and working locally.

## Quick Start

1. Prepare the env file and Isaac cache directories:

   ```bash
   cp .env.example .env
   mkdir -p ./.docker/isaac-sim/cache/{kit,ov,pip,glcache,computecache}
   mkdir -p ./.docker/isaac-sim/{logs,data,documents}
   ```

2. Build and start the K1 container:

   ```bash
   docker compose --env-file .env up --build -d k1-sim
   ```

   If your machine already has a local `k1-isaac-sim:5.0.0` image from the
   `ROSClaw-Vision` stack, you can set `ISAAC_SIM_BASE_IMAGE=k1-isaac-sim:5.0.0`
   in `.env` to reuse its bundled Isaac Lab install and shorten rebuilds.

3. Follow startup until Isaac and ROSClaw are both ready:

   ```bash
   docker compose --env-file .env logs -f k1-sim
   ```

   Wait for:

   ```text
   [entrypoint] Booster K1 runtime ready: Isaac streaming on :49100 and rosbridge is listening on :9090
   ```

   During startup, also watch for the slide-mode lines. You should see:

   ```text
   [entrypoint] Policy disabled for control mode motion_replay
   [motion_replay] creating interactive scene (freeze_motion=True start_frame=53 disable_gravity=True)
   ```

4. Connect the NVIDIA Isaac Sim WebRTC Streaming Client to the Docker host.

   - If the viewer is on the same machine, connect to `127.0.0.1`.
   - If the viewer is on another LAN machine, set `PUBLIC_IP` in `.env` before
     starting the container, then connect to that host.

5. Install the local ROSClaw plugin into OpenClaw:

   ```bash
   openclaw plugins install --link ../rosclaw-plugin
   ```

6. Configure the plugin to use rosbridge.

   Use [config/rosclaw-plugin.local.json](config/rosclaw-plugin.local.json) as
   the baseline:

   - `transport.mode = "rosbridge"`
   - `rosbridge.url = "ws://localhost:9090"`
   - `robot.namespace = "/k1"`

7. Chat with the OpenClaw agent.

   Example prompts:

   - `Move the Booster K1 forward slowly for one second, then stop.`
   - `Rotate the K1 to the left in place and stop when finished.`
   - `List the ROS topics available on the K1 before moving.`
   - `Check whether the Booster status topic reports simulator passthrough mode.`
   - `Stop the robot immediately.`

## What The Container Does

On startup the container does this in order:

1. Verifies this example is running in the expected `/k1` namespace.
2. Prepares any missing K1 motion assets with `prepare_k1_assets.py`.
3. Launches the Booster K1 Isaac Lab runtime in frozen `motion_replay` mode.
4. Disables policy loading for the default slide path.
5. Waits for the Isaac livestream port and the core K1 ROS topics.
6. Sources the system ROS 2 install and the built ROSClaw overlay.
7. Launches `ros2 launch rosclaw_bringup rosclaw.launch.py platform:=k1 rosbridge:=true perception:=false use_sim_time:=true`.
8. Waits for rosbridge and `/k1/booster/status`, then writes a readiness file.

ROSClaw talks to the simulated humanoid through rosbridge on port `9090`. The
K1 movement path is now a gravity-free `/k1/cmd_vel` slide, not policy-backed
walking or direct joint teleop.

## Verification

After the container is up, run:

```bash
./scripts/smoke_ready.sh
./scripts/smoke_livestream.sh
./scripts/smoke_rosbridge.sh
./scripts/smoke_topics.sh
./scripts/smoke_manifest.sh
./scripts/smoke_cmd_vel.sh
./scripts/smoke_motion.sh
./scripts/probe_stability.sh
```

The expected sequence is:

1. The readiness file reports `/k1` topics, confirms `motion_replay` mode, and
   confirms that policy loading is disabled for the default slide path.
2. Isaac livestream is listening on `:49100`.
3. rosbridge is reachable on `ws://localhost:9090`.
4. `/rosclaw/get_manifest` returns the K1 topics and status channel.
5. A bounded `/k1/cmd_vel` command is accepted by the runtime.
6. The K1 shows measurable planar motion on `/k1/odom`.
7. The stability probe can hold the robot at zero command and run a bounded
   sliding profile without tripping the configured near-fall thresholds.

## Notes

- Keep `K1_NAMESPACE=k1` for this example. The shipped ROSClaw K1 configs are
  aligned to the `/k1/*` topic layout.
- The default example settings are `K1_CONTROL_MODE=motion_replay`,
  `K1_FREEZE_MOTION=1`, `K1_MOTION_START_FRAME=53`, `K1_TELEOP_DEVICE=cmd_vel`,
  and `K1_SLIDE_DISABLE_GRAVITY=1`.
- In that default mode, the K1 does not use `walking_policy_latest.pt` or any
  other locomotion checkpoint. The readiness file should report blank policy
  fields.
- `K1_SDK_COMPAT=0` and `ROSCLAW_K1_SDK_ENABLED=0` are intentional. This is a
  simulator-path example; ROSClaw uses the ROS topics, not the Booster hardware
  SDK transport.
- The default slide path freezes the replay motion on frame `53` of
  `k1_mj2_seg1.npz`, zeros joint/root replay velocities, and moves the robot by
  writing the root state each frame from `/k1/cmd_vel`.
- Gravity is explicitly disabled on the spawned K1 for the default slide path,
  so the robot stays upright instead of trying to balance.
- This example does not enable Gemini perception or `/rosclaw/scene` by
  default. It focuses on simulator bring-up and stable slide motion.
- The older policy-backed `cmd_vel` path is still available in the repo, but it
  is no longer the default example behavior.
