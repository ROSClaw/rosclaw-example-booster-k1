#!/usr/bin/env bash
set -euo pipefail

ros_distro="${SYSTEM_ROS_DISTRO:-${ROS_DISTRO:-humble}}"
set +u
source "/opt/ros/${ros_distro}/setup.bash"
set -u

namespace="${K1_NAMESPACE:-k1}"
namespace="${namespace#/}"
namespace="${namespace%/}"

topic="${K1_CMD_VEL_TOPIC:-}"
if [[ -z "${topic}" ]]; then
    if [[ -n "${namespace}" ]]; then
        topic="/${namespace}/cmd_vel"
    else
        topic="/cmd_vel"
    fi
fi

echo "[teleop] Publishing Twist commands to ${topic}"
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:="${topic}"
