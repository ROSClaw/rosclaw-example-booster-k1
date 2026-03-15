#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_FILE="${ENV_FILE:-${ROOT_DIR}/.env}"
SERVICE_NAME="${SERVICE_NAME:-k1-sim}"

compose() {
    docker compose --env-file "${ENV_FILE}" -f "${ROOT_DIR}/compose.yaml" "$@"
}

container_bash() {
    compose exec -T "${SERVICE_NAME}" bash -lc "$1"
}

container_ros_bash() {
    container_bash "set -euo pipefail
set +u
source \"/opt/ros/\${SYSTEM_ROS_DISTRO:-\${ROS_DISTRO}}/setup.bash\"
source \"/workspace/rosclaw_ws/install/setup.bash\"
set -u
$1"
}
