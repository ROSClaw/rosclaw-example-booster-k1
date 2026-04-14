# DDS Host Match Report

Date: 2026-03-25

## Scope

This note documents:

- what ROS 2 topics, services, and actions are visible on the host after matching the host DDS config to the robot
- whether those topics / services / actions were verified to function from the host
- if not, why not, and what troubleshooting steps were attempted

The host is:

- `192.168.2.126`
- ROS 2 Humble
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`

The robot is:

- `booster@192.168.2.152`
- ROS 2 Humble
- mixed Fast DDS profiles:
  - most RPC bridges use `/opt/booster/BoosterRos2/fastdds_profile.xml`
  - `loco_rpc_bridge` uses `/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml`

## DDS Config Alignment

The robot's main Fast DDS profile is valid and whitelists:

- `127.0.0.1`
- `192.168.127.101`
- `192.168.2.152`

The robot does not use that profile uniformly.

The locomotion bridge is launched by:

- `/opt/booster/BoosterRos2/start_rpc_service.sh`

That script hardcodes:

- `FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml`

The UDP-only profile whitelists only:

- `127.0.0.1`
- `192.168.127.101`

It does not include the robot's Wi-Fi address `192.168.2.152`.

The copied profile outside this repo was malformed because it was missing the closing `</profiles>` tag.

To make the host match the robot configuration, two project-root files were created:

- `../dds-profile.xml`
- `../setup-dds-env.sh`

The project-root DDS profile adds the host IP `192.168.2.126` to the whitelist so the laptop can participate on `wlo1`.

The shell setup script:

- strips Conda-related Python environment variables and `PATH` entries
- sources `/opt/ros/humble/setup.bash`
- sources `real-hardware/install/setup.bash`
- exports `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- exports `FASTRTPS_DEFAULT_PROFILES_FILE` and `FASTDDS_DEFAULT_PROFILES_FILE` to the project-root `dds-profile.xml`

## Host-Visible Graph After Matching DDS

These were discovered from the host after sourcing `../setup-dds-env.sh`.

### Topics Seen On Host

- `/AiApiTopicReq` `[booster_msgs/msg/RpcReqMsg]`
- `/AiApiTopicResp` `[booster_msgs/msg/RpcRespMsg]`
- `/LightControlApiTopicReq` `[booster_msgs/msg/RpcReqMsg]`
- `/LightControlApiTopicResp` `[booster_msgs/msg/RpcRespMsg]`
- `/X5CameraControlReq` `[booster_msgs/msg/RpcReqMsg]`
- `/X5CameraControlResp` `[booster_msgs/msg/RpcRespMsg]`
- `/image_left_raw` `[sensor_msgs/msg/Image]`
- `/joy` `[sensor_msgs/msg/Joy]`
- `/joy/set_feedback` `[sensor_msgs/msg/JoyFeedback]`
- `/low_state` `[booster_interface/msg/LowState]`
- `/parameter_events` `[rcl_interfaces/msg/ParameterEvent]`
- `/rosout` `[rcl_interfaces/msg/Log]`

### Services Seen On Host

- `/booster_light_service` `[booster_interface/srv/RpcService]`
- `/booster_lui_service` `[booster_interface/srv/RpcService]`
- `/booster_rpc_service` `[booster_interface/srv/RpcService]`
- `/booster_rtc_service` `[booster_interface/srv/RpcService]`
- `/booster_vision_service` `[booster_interface/srv/RpcService]`
- `/booster_x5_camera_service` `[booster_interface/srv/RpcService]`
- parameter services for:
  - `/light_rpc_service_bridge`
  - `/lui_rpc_service_bridge`
  - `/rtc_rpc_service_bridge`
  - `/x5_camera_rpc_service_bridge`

### Actions Seen On Host

- none

## Robot-Side Ground Truth

The robot itself has a richer graph than what the host currently sees.

### Robot Topics Confirmed

- `/low_state`
- `/joint_ctrl`
- `/joint_states`
- `/LocoApiTopicReq`
- `/LocoApiTopicResp`
- `/AiApiTopicReq`
- `/AiApiTopicResp`
- `/LightControlApiTopicReq`
- `/LightControlApiTopicResp`
- `/X5CameraControlReq`
- `/X5CameraControlResp`
- `/image_left_raw`
- many additional perception and state topics

### Robot Services Confirmed

- `/booster_rpc_service`
- `/booster_rtc_service`
- `/booster_light_service`
- `/booster_lui_service`
- `/booster_vision_service`
- `/booster_x5_camera_service`

### Robot Actions Confirmed

- none

## Functional Status

### Services

#### Read-only parameter service

Status: working from host

Safe test executed from the host:

```bash
ros2 service call /rtc_rpc_service_bridge/get_parameter_types \
  rcl_interfaces/srv/GetParameterTypes \
  "{names: [\"use_sim_time\"]}"
```

Observed response:

```text
types=[1]
```

This verifies that at least standard ROS 2 service request/response traffic is working from the host to the robot after matching the DDS config.

#### Booster RPC services

Status: discovered on host, not command-tested

Visible from the host:

- `/booster_rpc_service`
- `/booster_rtc_service`
- `/booster_light_service`
- `/booster_lui_service`
- `/booster_vision_service`
- `/booster_x5_camera_service`

Reason not function-tested:

- these services accept app-specific `RpcService` requests containing API IDs plus JSON payloads
- there is no documented no-op or guaranteed harmless request contract in this workspace
- calling the locomotion or manipulation APIs without a known safe request shape could move hardware

Conclusion:

- service discovery works
- a safe standard service call works
- the main Booster command services are reachable at discovery level, but actuator-side behavior was not validated here

### Topics

#### `/low_state`

Status: discovered on host, data not received on host

What was verified:

- the host can discover `/low_state`
- the robot confirms `/low_state` has a live publisher
- on the robot, the `/low_state` publisher appears as `_CREATED_BY_BARE_DDS_APP_`
- the robot reports the publisher QoS as:
  - reliability: `RELIABLE`
  - durability: `TRANSIENT_LOCAL`
- the robot can also see the host subscription when the host tries to echo the topic

What did not work:

- host `ros2 topic echo /low_state ... --once` did not receive a sample
- host `ros2 topic info /low_state -v` still showed `Publisher count: 0`

Conclusion:

- `/low_state` is definitely being published on the robot
- the host can advertise a subscription to it
- but the host still is not receiving topic data

#### `/image_left_raw`

Status: discovered on host, data not received on host

What did not work:

- host `ros2 topic echo /image_left_raw ... --once` did not receive a sample
- host `ros2 topic info /image_left_raw -v` also showed `Publisher count: 0`

Conclusion:

- the problem is not unique to `/low_state`
- host topic data reception is still incomplete even after DDS discovery was fixed

#### `/joy`, `/AiApiTopicReq`, `/AiApiTopicResp`, `/LightControlApiTopicReq`, `/LightControlApiTopicResp`, `/X5CameraControlReq`, `/X5CameraControlResp`

Status: discovered only

These topics were discovered on the host, but they were not individually sampled.

### Actions

Status: none visible

No actions were visible on either the robot or the host during testing.

## Troubleshooting Steps Attempted

1. Compared the host and robot network addresses.
   - Host: `192.168.2.126`
   - Robot: `192.168.2.152`
2. Verified the robot's active Fast DDS profile on the robot over SSH.
3. Found the copied DDS profile was malformed because it was missing `</profiles>`.
4. Created a valid project-root `dds-profile.xml`.
5. Added the host IP `192.168.2.126` to the profile whitelist.
6. Created `setup-dds-env.sh` to remove Conda interference and force Fast DDS.
7. Re-ran host discovery with that setup.
8. Verified that the host can now discover the main Booster services and several robot topics.
9. Verified from the robot that `/low_state` is actively published.
10. Verified from the robot that the robot sees the host subscription to `/low_state`.
11. Retried `/low_state` from the host with explicit message type and matching QoS:
    - `--qos-reliability reliable`
    - `--qos-durability transient_local`
12. Repeated a similar topic data test against `/image_left_raw`.
13. Executed a safe, read-only parameter service call from the host and received a response.
14. Executed host-side RPC smoke tests against the Booster services with `rclpy`.
15. Verified on the robot that the same safe `kGetMode` request succeeds locally against `/booster_rpc_service`.
16. Traced the `loco_rpc_bridge` process tree on the robot back to `/opt/booster/BoosterRos2/start_rpc_service.sh`.
17. Verified that `start_rpc_service.sh` exports `/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml`, and that this profile does not whitelist `192.168.2.152`.
18. Backed up `/opt/booster/BoosterRos2/start_rpc_service.sh` on the robot and changed it to export `/opt/booster/BoosterRos2/fastdds_profile.xml`.
19. Restarted the locomotion bridge and verified the new `loco_rpc_bridge` process environment uses `/opt/booster/BoosterRos2/fastdds_profile.xml`.
20. Observed that `start_rpc_service.sh` begins with `killall rpc_service_node`, which also killed the RTC, light, LUI, and X5 camera bridge nodes during the manual relaunch.
21. Manually relaunched `rpc_bridge_launch_perception.py` on the robot with the full Fast DDS profile to restore the non-locomotion RPC services.
22. Verified from the host that all six Booster RPC services are again visible after the relaunch.
23. Verified from the host that safe `/booster_rpc_service` getter requests now return valid responses.
24. Re-tested `/low_state` from the host after the profile unification and still received no sample; `ros2 topic info /low_state -v` still showed `Publisher count: 0`.
25. Verified on the robot that `/low_state`, `/joint_ctrl`, and `/joint_states` are exposed by bare DDS endpoints rather than normal ROS 2 nodes.
26. Compared their endpoint GIDs and found they share the same participant prefix:
    - `/low_state` publisher: `...d8.0f.22...`
    - `/joint_states` publisher: `...d8.0f.22...`
    - `/joint_ctrl` subscriptions: `...d8.0f.22...`
27. Checked the native motion binaries on the robot and found that:
    - `/opt/booster/BoosterServer/bin/booster-server`
    - `/opt/booster/DeviceGateway/bin/device-gateway`
    - `/opt/booster/Gait/bin/booster-motion`
    all embed `FASTRTPS_DEFAULT_PROFILES_FILE` and `/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml`
28. Verified that the running native motion-stack processes do not expose `FASTRTPS_DEFAULT_PROFILES_FILE` in `/proc/<pid>/environ`, unlike the ROS bridge nodes that do expose the full profile there.
29. Added a robot-local ROS 2 relay package in this workspace at `src/k1_low_level_relay`.
30. Built that package locally in this workspace and built it again on the K1 in `/tmp/k1-relay-ws`.
31. Ran the relay on the K1 with the full Fast DDS profile and verified from the host that the relay published:
    - `/k1/low_state`
    - `/k1/joint_states`
    - `/k1/joint_ctrl`
    - `/k1/low_cmd`
32. Verified from the host that `/k1/low_state` delivered a real `booster_interface/msg/LowState` sample.
33. Verified from the host that `/k1/joint_states` delivered a real `sensor_msgs/msg/JointState` sample.
34. Left the low-level command relay path untested on hardware to avoid sending actuator commands unnecessarily.

## Current Best Explanation

The DDS discovery problem on the host was real and was fixed by:

- correcting the XML
- adding the host IP to the whitelist
- removing Conda from the active Python environment
- forcing Fast DDS

After that fix:

- service discovery works
- topic discovery works
- a read-only service call succeeds

The locomotion RPC failure was caused by a robot-side DDS profile mismatch and is fixed when all RPC bridges use the same full Fast DDS profile.

- `/booster_rpc_service` is served by `loco_rpc_bridge`
- `loco_rpc_bridge` was launched by `/opt/booster/BoosterRos2/start_rpc_service.sh`
- that script originally forced `/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml`
- that profile omitted the robot's Wi-Fi address `192.168.2.152`
- after changing `start_rpc_service.sh` to use `/opt/booster/BoosterRos2/fastdds_profile.xml`, host calls to `/booster_rpc_service` returned valid getter responses

This shows the control-path problem was a Fast DDS configuration inconsistency, not a Conda issue and not a host-side SDK problem.

What remains unresolved is host topic data reception.

For topic data, the remaining issue is still most likely one of:

- incomplete topic endpoint matching between the host ROS 2 tools and the robot publishers
- interoperability issues with publishers exposed as bare DDS apps rather than normal ROS 2 nodes
- a remaining DDS transport or QoS mismatch that affects topic data more than service discovery
- a separate hardcoded or internally loaded UDP-only Fast DDS profile in the native motion stack

This is why the host can now discover `/low_state` and several services, and can successfully call some RPC services, but still cannot receive topic samples from `/low_state` or `/image_left_raw`.

There is now strong evidence that the low-level bare DDS topics do use a different Fast DDS configuration path from the ROS bridge nodes:

- the RPC bridges use `/opt/booster/BoosterRos2/fastdds_profile.xml` through their process environment
- the low-level bare DDS participant behind `/low_state`, `/joint_ctrl`, and `/joint_states` appears to come from the native motion stack
- the native motion binaries embed `/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml`
- those native processes do not advertise the profile through environment variables, so the profile is likely hardcoded or loaded internally

## Practical Outcome

Working now:

- host discovery of the robot's main Booster services
- host discovery of several robot topics
- safe read-only ROS 2 service traffic from host to robot
- host RPC calls to `/booster_rpc_service` safe getters
- host RPC calls to `/booster_x5_camera_service`, `/booster_rtc_service`, `/booster_lui_service`, and `/booster_light_service`
- robot-local relay of bare DDS state topics to ROS 2 topics under `/k1`
- host receipt of relayed state data on `/k1/low_state` and `/k1/joint_states`

Not yet working:

- confirmed host receipt of topic payloads from robot topics such as `/low_state`
- consistent host topic delivery for `/low_state` and `/image_left_raw`, even after putting all robot RPC bridges on the same Fast DDS profile
- unifying the bare DDS motion-stack topics with the same full Fast DDS profile as the ROS bridge nodes

## RPC Smoke Test From Host

A host-side smoke test script was added at:

- `rpc_service_smoke_test.py`

It uses:

- `rclpy`
- `booster_interface/srv/RpcService`
- the project-root `setup-dds-env.sh`

### Requests attempted

- `/booster_rpc_service`
  - `api_id=2017` `loco.get_mode`
  - `api_id=2018` `loco.get_status`
  - `api_id=2022` `loco.get_robot_info`
- `/booster_vision_service`
  - `api_id=3002` `vision.get_detection_object`
- `/booster_x5_camera_service`
  - `api_id=5002` `x5.get_status`
- `/booster_rtc_service`
  - `api_id=2001` `rtc.stop_ai_chat`
- `/booster_lui_service`
  - `api_id=1051` `lui.stop_tts`
- `/booster_light_service`
  - `api_id=2001` `light.stop_led`

### Results

#### `/booster_rpc_service`

Status: callable from host after robot profile unification

Observed behavior before the robot-side profile fix:

- service discovery could list `/booster_rpc_service`
- the host `rclpy` client still reported `service not available`
- a direct `ros2 service call` to `/booster_rpc_service` with a safe `kGetMode` request also failed to establish availability before timeout

Robot-side root cause:

- the same safe `kGetMode` request succeeds locally on the robot
- local robot response was `status: 0`
- local robot response body was `{"mode":2}`
- the `loco_rpc_bridge` process environment contains:
  - `FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml`
- the parent launcher is:
  - `/usr/bin/python3 /opt/ros/humble/bin/ros2 launch booster_rpc_bridge rpc_bridge_launch_motion.py`
- the launch parent is:
  - `/bin/bash /opt/booster/BoosterRos2/start_rpc_service.sh -log_dir ...`
- `start_rpc_service.sh` explicitly exports the UDP-only profile above
- that UDP-only profile does not whitelist `192.168.2.152`

Observed behavior after the robot-side profile fix:

- `loco.get_mode` returned `status: 0`, `body: {"mode":2}` in the first host retest after the fix
- `loco.get_status` returned `status: 0`
- `loco.get_robot_info` returned `status: 0`
- in one later smoke-test run, the first getter hit `service not available`, but later getters in the same run succeeded

Current interpretation:

- the `/booster_rpc_service` host control path now works when `loco_rpc_bridge` uses the same full Fast DDS profile as the other RPC bridges
- the remaining occasional first-call miss looks like startup or discovery timing, not the earlier hard transport failure

#### `/booster_vision_service`

Status: inconsistent from host

Observed response:

- `status: -1`
- `body: {"error": "Service not started"}`
- on a later retry, the host timed out waiting for the response

Interpretation:

- the host can at least discover the service and has seen one application-level response
- the backend is not in a healthy started state, and responsiveness was not stable across retries

#### `/booster_x5_camera_service`

Status: callable from host

Observed response:

- `status: 0`
- `body: {"status":0}`

Interpretation:

- the service is reachable from the host
- the camera backend returned a valid status payload

#### `/booster_rtc_service`

Status: callable from host

Observed response:

- `status: 0`
- empty body

Interpretation:

- the RTC RPC bridge is reachable and processed the request

#### `/booster_lui_service`

Status: callable from host

Observed response:

- `status: 0`
- empty body

Interpretation:

- the LUI RPC bridge is reachable and processed the request

#### `/booster_light_service`

Status: callable from host

Observed response:

- `status: 0`
- `body: StopLEDLightControl done`

Interpretation:

- the light RPC bridge is reachable and returned an application-level success response

### Summary of RPC test

Confirmed callable from host:

- `/booster_rpc_service` safe getters
- `/booster_x5_camera_service`
- `/booster_rtc_service`
- `/booster_lui_service`
- `/booster_light_service`
- `/booster_vision_service` (transport works; backend reported not started)

Not callable from host during this session:

- none of the tested services remained completely unreachable after the profile unification

This means the host can now use the main Booster control service through the ROS 2 SDK path, as long as the robot launches `loco_rpc_bridge` with the same full Fast DDS profile as the other RPC bridges.

## Relay Package Result

A relay package was added in this workspace at:

- `src/k1_low_level_relay`

The node runs on the robot and bridges:

- `/low_state` -> `/k1/low_state`
- `/joint_states` -> `/k1/joint_states`
- `/k1/joint_ctrl` -> `/joint_ctrl`
- `/k1/low_cmd` -> `/joint_ctrl`

Result:

- the relay approach works for state telemetry
- the host was able to discover `/k1/low_state`, `/k1/joint_states`, `/k1/joint_ctrl`, and `/k1/low_cmd`
- the host successfully received real samples from `/k1/low_state`
- the host successfully received real samples from `/k1/joint_states`
- the low-level command relay topics were exposed, but command transmission was not validated on hardware in this session

## Suggested Next Steps

1. Keep the robot locomotion bridge on `/opt/booster/BoosterRos2/fastdds_profile.xml`; that change fixed host access to `/booster_rpc_service`.
2. Validate the robot's normal startup path after a clean restart, because the manual relaunch path used here temporarily killed the perception bridge nodes and required a manual relaunch of `rpc_bridge_launch_perception.py`.
3. Continue topic debugging for `/low_state` and `/image_left_raw`; the full Fast DDS profile unification did not make those topics deliver samples to the host.
4. Inspect the native motion stack, especially `booster-motion`, `device-gateway`, and `booster-server`, for where they load `/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml`, then switch that path to the full profile or a new shared profile.
5. If you want a lower-risk workaround before changing the native motion stack, keep using the robot-local relay node in `k1_low_level_relay` to expose namespaced ROS 2 topics like `/k1/low_state` and `/k1/joint_states`.
6. If you want command control through the relay, validate `/k1/joint_ctrl` or `/k1/low_cmd` carefully on hardware once the robot is in a safe state.
7. If you still want to trial CycloneDDS, audit which robot publishers are true ROS 2 nodes versus DDS-native processes first. `/low_state` appears as `_CREATED_BY_BARE_DDS_APP_`, so switching only the ROS 2 RMW may not move that publisher to CycloneDDS.

## April 2026 K1 Service Loop Fix

Observed on April 15, 2026 on the live K1:

- repeated torque disable and re-enable with the motion boot sound
- repeated restarts of locomotion-facing services without a full Jetson reboot
- kernel and systemd reports showing OOM kills across the Booster ROS 2 stack

The immediate offenders were the robot-side ROS 2 processes launched by
Booster's shell scripts, especially `rpc_service_node`, `robot_state_publisher`,
and `joy_node`. On this K1, the SHM-enabled profile
`/opt/booster/BoosterRos2/fastdds_profile.xml` correlated with:

- `RTPS_TRANSPORT_SHM` initialization errors
- `ParticipantEntitiesInfo` `Bad alloc` errors
- large transient RSS growth in the ROS 2 bridge processes
- `booster-daemon.service` being OOM-killed and restarted, which retriggered
  motor connect and produced the audible loop

The fix that stabilized the robot was to switch the active Booster startup
scripts to the existing UDP-only profile
`/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml` and then restart the
affected services.

This supersedes the earlier suggestion in this report to keep the locomotion
bridge on `fastdds_profile.xml`. For this robot, the stable configuration was
the UDP-only profile across the active Booster ROS 2 startup scripts.

Use the repo-side helper to reapply and verify the fix:

```bash
./real-hardware/fix_k1_fastdds_udp_only.sh
```

For a non-mutating inspection:

```bash
./real-hardware/fix_k1_fastdds_udp_only.sh --check-only
```
