#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"

container_ros_bash '
topics="$(ros2 topic list --no-daemon)"
printf "%s\n" "${topics}"

required_topics=(
  /k1/clock
  /k1/cmd_vel
  /k1/odom
  /k1/imu
  /k1/joint_states
  /k1/front_cam/rgb
  /k1/front_cam/camera_info
  /k1/booster/status
)

for topic in "${required_topics[@]}"; do
    if ! grep -qxF "${topic}" <<<"${topics}"; then
        echo "missing expected topic: ${topic}" >&2
        exit 1
    fi
done
'
