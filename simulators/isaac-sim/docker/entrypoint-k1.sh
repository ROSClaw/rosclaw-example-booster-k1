#!/usr/bin/env bash
set -euo pipefail

export ACCEPT_EULA="${ACCEPT_EULA:-Y}"
export PRIVACY_CONSENT="${PRIVACY_CONSENT:-Y}"
export LIVESTREAM="${LIVESTREAM:-2}"
export PUBLIC_IP="${PUBLIC_IP:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export SYSTEM_ROS_DISTRO="${SYSTEM_ROS_DISTRO:-${ROS_DISTRO:-humble}}"
export ISAACSIM_INTERNAL_ROS_DISTRO="${ISAACSIM_INTERNAL_ROS_DISTRO:-jazzy}"
export ISAAC_LIVESTREAM_PORT="${ISAAC_LIVESTREAM_PORT:-49100}"
export ROSCLAW_READY_FILE="${ROSCLAW_READY_FILE:-/tmp/rosclaw-k1-ready.json}"
export K1_STATE_FILE="${K1_STATE_FILE:-/tmp/rosclaw-k1-state.json}"
export DEFAULT_SCENE_USD="${DEFAULT_SCENE_USD:-/Isaac/Environments/Simple_Warehouse/full_warehouse.usd}"

normalize_namespace() {
    local raw="${1:-}"
    raw="${raw#/}"
    raw="${raw%/}"
    printf '%s' "${raw}"
}

source_system_ros() {
    set +u
    if [[ -f "/opt/ros/${SYSTEM_ROS_DISTRO}/setup.bash" ]]; then
        # shellcheck source=/dev/null
        source "/opt/ros/${SYSTEM_ROS_DISTRO}/setup.bash"
    fi
    if [[ -f "/workspace/booster_sdk_ws/install/setup.bash" ]]; then
        # shellcheck source=/dev/null
        source "/workspace/booster_sdk_ws/install/setup.bash"
    fi
    if [[ -f "/workspace/rosclaw_ws/install/setup.bash" ]]; then
        # shellcheck source=/dev/null
        source "/workspace/rosclaw_ws/install/setup.bash"
    fi
    set -u
}

detect_isaac_ros_root() {
    local candidate
    for candidate in \
        "/isaac-sim/exts/isaacsim.ros2.bridge/${SYSTEM_ROS_DISTRO}" \
        "/isaac-sim/exts/isaacsim.ros2.bridge/${ISAACSIM_INTERNAL_ROS_DISTRO}" \
        "/isaac-sim/exts/isaacsim.ros2.bridge/humble" \
        "/isaac-sim/exts/isaacsim.ros2.bridge/jazzy"; do
        if [[ -d "${candidate}/rclpy" && -d "${candidate}/lib" ]]; then
            printf '%s' "${candidate}"
            return 0
        fi
    done
    return 1
}

ensure_process_alive() {
    local pid="${1:-}"
    local label="$2"
    if [[ -z "${pid}" ]]; then
        return 0
    fi
    if kill -0 "${pid}" 2>/dev/null; then
        return 0
    fi
    echo "[entrypoint] ${label} exited before startup completed" >&2
    wait "${pid}"
}

wait_for_port() {
    local port="$1"
    local label="$2"
    local max_tries="${3:-120}"
    for _ in $(seq 1 "${max_tries}"); do
        ensure_process_alive "${isaac_pid:-}" "Isaac runtime" || return 1
        if [[ -n "${rosclaw_pid:-}" ]]; then
            ensure_process_alive "${rosclaw_pid:-}" "ROSClaw bringup" || return 1
        fi
        if ss -ltn 2>/dev/null | grep -q ":${port}\\b"; then
            echo "[entrypoint] ${label} ready on :${port}"
            return 0
        fi
        sleep 1
    done
    echo "[entrypoint] ${label} did not start on :${port}" >&2
    return 1
}

wait_for_file() {
    local path="$1"
    local label="$2"
    local max_tries="${3:-180}"
    for _ in $(seq 1 "${max_tries}"); do
        ensure_process_alive "${isaac_pid:-}" "Isaac runtime" || return 1
        if [[ -f "${path}" ]]; then
            echo "[entrypoint] ${label} ready at ${path}"
            return 0
        fi
        sleep 1
    done
    echo "[entrypoint] ${label} did not appear at ${path}" >&2
    return 1
}

cleanup() {
    if [[ -n "${isaac_pid:-}" ]] && kill -0 "${isaac_pid}" 2>/dev/null; then
        kill "${isaac_pid}" 2>/dev/null || true
        wait "${isaac_pid}" 2>/dev/null || true
    fi
    if [[ -n "${rosclaw_pid:-}" ]] && kill -0 "${rosclaw_pid}" 2>/dev/null; then
        kill "${rosclaw_pid}" 2>/dev/null || true
        wait "${rosclaw_pid}" 2>/dev/null || true
    fi
}

trap cleanup EXIT INT TERM

if [[ $# -gt 0 ]]; then
    source_system_ros
    exec "$@"
fi

namespace="$(normalize_namespace "${K1_NAMESPACE:-k1}")"
if [[ -z "${namespace}" ]]; then
    echo "[entrypoint] K1_NAMESPACE must not be empty" >&2
    exit 1
fi
export K1_NAMESPACE="${namespace}"
export K1_SCENE_USD="${K1_SCENE_USD:-${DEFAULT_SCENE_USD}}"

rm -f "${ROSCLAW_READY_FILE}" "${K1_STATE_FILE}"

isaac_args=(
    "--headless"
    "--namespace" "${K1_NAMESPACE}"
    "--scene-usd" "${K1_SCENE_USD}"
    "--policy-path" "${K1_POLICY_CONTAINER_PATH:-/workspace/policies/k1_cmd_vel.pt}"
    "--publish-period-ms" "${K1_CONTROL_PERIOD_MS:-20}"
    "--policy-decimation" "${K1_POLICY_DECIMATION:-4}"
    "--obs-dim" "${K1_POLICY_OBS_DIM:-255}"
    "--height-scan-dim" "${K1_POLICY_HEIGHT_SCAN_DIM:-187}"
    "--robot-z" "${K1_ROBOT_Z:-0.57}"
)

if [[ -n "${LIVESTREAM:-}" ]]; then
    isaac_args+=("--livestream" "${LIVESTREAM}")
fi

echo "[entrypoint] Starting Booster K1 warehouse runtime"
isaac_ros_root="$(detect_isaac_ros_root || true)"
declare -a isaac_env=()
if [[ -n "${isaac_ros_root}" ]]; then
    echo "[entrypoint] Using Isaac internal ROS bundle at ${isaac_ros_root}"
    isaac_env+=(
        "ROS_DISTRO=${SYSTEM_ROS_DISTRO}"
        "PYTHONPATH=${isaac_ros_root}/rclpy${PYTHONPATH:+:${PYTHONPATH}}"
        "LD_LIBRARY_PATH=${isaac_ros_root}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
    )
fi

env "${isaac_env[@]}" /isaac-sim/python.sh /workspace/rosclaw-example-booster-k1/simulators/isaac-sim/app/k1_warehouse_demo.py "${isaac_args[@]}" &
isaac_pid=$!

if [[ "${LIVESTREAM}" != "0" ]]; then
    wait_for_port "${ISAAC_LIVESTREAM_PORT}" "Isaac livestream" 180
fi

wait_for_file "${ROSCLAW_READY_FILE}" "K1 runtime readiness" 180

source_system_ros

echo "[entrypoint] Launching ROSClaw bringup"
ros2 launch rosclaw_bringup k1.launch.py rosbridge:=true perception:=false use_sim_time:=true &
rosclaw_pid=$!

wait_for_port 9090 "rosbridge" 90

if [[ "${LIVESTREAM}" != "0" ]]; then
    echo "[entrypoint] Booster K1 runtime ready: Isaac streaming on :${ISAAC_LIVESTREAM_PORT} and rosbridge is listening on :9090"
else
    echo "[entrypoint] Booster K1 runtime ready: Isaac running and rosbridge is listening on :9090"
fi

while true; do
    ensure_process_alive "${isaac_pid}" "Isaac runtime" || exit 1
    ensure_process_alive "${rosclaw_pid}" "ROSClaw bringup" || exit 1
    sleep 2
done
