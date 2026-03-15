#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"

container_ros_bash '
manifest_output="$(ros2 service call /rosclaw/get_manifest rosclaw_msgs/srv/GetManifest "{robot_namespace: \"/k1\"}")"
printf "%s\n" "${manifest_output}"

for expected in "/k1/cmd_vel" "/k1/odom" "/k1/front_cam/rgb" "/k1/booster/status"; do
    if ! grep -q "${expected}" <<<"${manifest_output}"; then
        echo "manifest missing expected entry: ${expected}" >&2
        exit 1
    fi
done
'
