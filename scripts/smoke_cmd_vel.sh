#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"

LINEAR_X="${LINEAR_X:-0.12}"
ANGULAR_Z="${ANGULAR_Z:-0.00}"
DURATION_SEC="${DURATION_SEC:-0.75}"
PUBLISH_RATE="${PUBLISH_RATE:-10}"

container_ros_bash "
set +e
timeout ${DURATION_SEC}s ros2 topic pub -r ${PUBLISH_RATE} /k1/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: ${LINEAR_X}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${ANGULAR_Z}}}'
status=\$?
set -e
if [[ \${status} -ne 0 && \${status} -ne 124 ]]; then
    exit \${status}
fi
ros2 topic pub --once /k1/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
echo 'bounded cmd_vel publish completed'
"
