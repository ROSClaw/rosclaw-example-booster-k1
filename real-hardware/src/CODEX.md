Booster K1 ROS2 SDK

This is a ROS2 interface SDK for the Booster K1 quadruped robot. Yes — you can absolutely send commands via ROS2 to control the robot.

Architecture

The SDK has two packages:

┌────────────────────────┬──────────────────────────────────────────────────────┐
│        Package         │                       Purpose                        │
├────────────────────────┼──────────────────────────────────────────────────────┤
│ booster_ros2_interface │ Message (20 types) and service (2 types) definitions │
├────────────────────────┼──────────────────────────────────────────────────────┤
│ booster_ros2_example   │ Example nodes in C++ and Python                      │
└────────────────────────┴──────────────────────────────────────────────────────┘

Communication happens primarily through ROS2 service calls using JSON-serialized API requests:

- booster_rpc_service — locomotion, hand/arm control, mode switching
- booster_rtc_service — AI chat, speech, face tracking

What You Can Control

Locomotion:
- kMove — velocity commands (vx, vy, vyaw)
- kLieDown / kGetUp — posture transitions
- kChangeMode — switch operational modes
- kRotateHead — head pitch/yaw

Manipulation:
- kMoveHandEndEffector — move hand to target pose
- kControlGripper — gripper open/close with force control
- kSwitchHandEndEffectorControlMode — enable/disable hand control

AI/Speech:
- kStartAiChat / kStopAiChat — conversational AI
- kSpeak — text-to-speech
- kStartFaceTracking / kStopFaceTracking

Low-level:
- Subscribe to /low_state for motor states, IMU, odometry
- Publish LowCmd for direct motor control (position, velocity, torque, gains)

Quick Start

colcon build
source install/setup.bash

# Run the RPC client example
ros2 run booster_rpc_client rpc_client_node

# Or the low-level state subscriber
ros2 run low_level low_level_subscriber_node

Example nodes in both C++ (rpc_client/src/client.cpp) and Python (rpc_client/src/client.py) demonstrate the full workflow — switching modes, sending velocity
commands, and moving the hand end-effector.
