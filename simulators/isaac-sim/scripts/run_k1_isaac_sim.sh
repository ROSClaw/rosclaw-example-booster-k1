#!/usr/bin/env bash
set -euo pipefail

MODE="webrtc"
IMAGE="${IMAGE:-rosclaw-example-booster-k1:0.3.0}"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_ROOT="$(cd "${ROOT_DIR}/../.." && pwd)"
K1_PORTED_RUNTIME_HOST_ROOT="${K1_PORTED_RUNTIME_HOST_ROOT:-${REPO_ROOT}/isaac-sim-runtime}"
K1_PORTED_RUNTIME_CONTAINER_ROOT="${K1_PORTED_RUNTIME_CONTAINER_ROOT:-/workspace/rosclaw-example-booster-k1/isaac-sim-runtime}"
DATA_DIR="${ROOT_DIR}/.docker/isaac-sim"
HOSTNAME_VALUE="${HOSTNAME_VALUE:-$(hostname)}"
DEFAULT_SCENE_USD="/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
PUBLIC_IP="${PUBLIC_IP:-}"
K1_NAMESPACE="${K1_NAMESPACE:-k1}"
K1_POLICY_HOST_PATH="${K1_POLICY_HOST_PATH:-}"
K1_POLICY_CONTAINER_PATH="${K1_POLICY_CONTAINER_PATH:-/workspace/policies/k1_cmd_vel.pt}"
K1_SCENE_USD="${K1_SCENE_USD:-${DEFAULT_SCENE_USD}}"
K1_CONTROL_PERIOD_MS="${K1_CONTROL_PERIOD_MS:-20}"
K1_POLICY_DECIMATION="${K1_POLICY_DECIMATION:-4}"
K1_POLICY_OBS_DIM="${K1_POLICY_OBS_DIM:-255}"
K1_POLICY_HEIGHT_SCAN_DIM="${K1_POLICY_HEIGHT_SCAN_DIM:-187}"
K1_POLICY_OBS_MODE="${K1_POLICY_OBS_MODE:-flat_height_scan}"
K1_POLICY_COMMAND_SCALE="${K1_POLICY_COMMAND_SCALE:-1.0}"
K1_POLICY_COMMAND_OFFSET="${K1_POLICY_COMMAND_OFFSET:-0.0}"
K1_POLICY_ACTION_GAIN="${K1_POLICY_ACTION_GAIN:-1.0}"
K1_MANUAL_STANDING_RESET="${K1_MANUAL_STANDING_RESET:-1}"
K1_REMAP_POLICY_ACTIONS="${K1_REMAP_POLICY_ACTIONS:-0}"
K1_ROBOT_Z="${K1_ROBOT_Z:-0.57}"
K1_ROBOT_ASSET_MODE="${K1_ROBOT_ASSET_MODE:-full}"
K1_CONTROLLER_MODE="${K1_CONTROLLER_MODE:-kinematic_gait}"
K1_KINEMATIC_DT_MODE="${K1_KINEMATIC_DT_MODE:-wall}"
K1_KINEMATIC_MAX_DT="${K1_KINEMATIC_MAX_DT:-0.05}"
K1_KINEMATIC_SPEED_SCALE="${K1_KINEMATIC_SPEED_SCALE:-1.0}"
K1_KINEMATIC_LIN_X_MIN="${K1_KINEMATIC_LIN_X_MIN:--0.8}"
K1_KINEMATIC_LIN_X_MAX="${K1_KINEMATIC_LIN_X_MAX:-0.8}"
K1_VIEW_CAMERA_MODE="${K1_VIEW_CAMERA_MODE:-observer}"
K1_VIEW_CAMERA_EYE="${K1_VIEW_CAMERA_EYE:-2.6,-3.2,1.35}"
K1_VIEW_CAMERA_TARGET="${K1_VIEW_CAMERA_TARGET:-0.0,0.0,0.55}"
K1_VIEW_CAMERA_FOCAL_LENGTH="${K1_VIEW_CAMERA_FOCAL_LENGTH:-28.0}"
K1_BOOSTER_ASSET_OVERRIDE="${K1_BOOSTER_ASSET_OVERRIDE:-}"
K1_APP_OVERRIDE="${K1_APP_OVERRIDE:-${ROOT_DIR}/k1_warehouse_demo_override.py}"
K1_ENTRYPOINT_OVERRIDE="${K1_ENTRYPOINT_OVERRIDE:-${ROOT_DIR}/docker/entrypoint-k1.sh}"
CONTAINER_NAME=""

normalize_namespace() {
    local raw="${1:-}"
    raw="${raw#/}"
    raw="${raw%/}"
    printf '%s' "${raw}"
}

usage() {
    cat <<'EOF'
Usage: ./scripts/run_k1_isaac_sim.sh [--mode webrtc|gui|headless] [--name CONTAINER]

Modes:
  webrtc   Start the K1 Isaac Sim stack with Isaac Sim WebRTC enabled.
  gui      Start the K1 Isaac Sim runtime with a local Isaac Sim GUI window.
  headless Start the K1 Isaac Sim stack headless without WebRTC.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --mode)
            MODE="$2"
            shift 2
            ;;
        --name)
            CONTAINER_NAME="$2"
            shift 2
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            echo "[run_k1_isaac_sim] unknown argument: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

K1_NAMESPACE="$(normalize_namespace "${K1_NAMESPACE}")"
if [[ -z "${K1_NAMESPACE}" ]]; then
    echo "[run_k1_isaac_sim] K1_NAMESPACE must not be empty" >&2
    exit 1
fi

if [[ ! -d "${K1_PORTED_RUNTIME_HOST_ROOT}" ]]; then
    echo "[run_k1_isaac_sim] ported runtime tree not found: ${K1_PORTED_RUNTIME_HOST_ROOT}" >&2
    exit 1
fi

case "${MODE}" in
    webrtc)
        LIVESTREAM="2"
        CONTAINER_NAME="${CONTAINER_NAME:-rosclaw-example-booster-k1-webrtc}"
        ;;
    gui)
        LIVESTREAM="0"
        CONTAINER_NAME="${CONTAINER_NAME:-rosclaw-example-booster-k1-gui}"
        ;;
    headless)
        LIVESTREAM="0"
        CONTAINER_NAME="${CONTAINER_NAME:-rosclaw-example-booster-k1-headless}"
        ;;
    *)
        echo "[run_k1_isaac_sim] unsupported mode: ${MODE}" >&2
        exit 1
        ;;
esac

mkdir -p \
    "${DATA_DIR}/cache/kit" \
    "${DATA_DIR}/cache/ov" \
    "${DATA_DIR}/cache/pip" \
    "${DATA_DIR}/cache/glcache" \
    "${DATA_DIR}/cache/computecache" \
    "${DATA_DIR}/logs" \
    "${DATA_DIR}/data" \
    "${DATA_DIR}/documents"

mapfile -t existing_names < <(docker ps -a --format '{{.Names}}')
if printf '%s\n' "${existing_names[@]}" | grep -qx "${CONTAINER_NAME}"; then
    docker rm -f "${CONTAINER_NAME}" >/dev/null
fi

docker_args=(
    run
    --rm
    --name "${CONTAINER_NAME}"
    --gpus all
    --network host
    --ipc host
    --security-opt label=disable
    --hostname "${HOSTNAME_VALUE}"
    --tty
    --interactive
    -e ACCEPT_EULA=Y
    -e PRIVACY_CONSENT=Y
    -e NVIDIA_VISIBLE_DEVICES=all
    -e NVIDIA_DRIVER_CAPABILITIES=all
    -e VK_DRIVER_FILES=/etc/vulkan/icd.d/nvidia_icd.json
    -e OMNI_SERVER=https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0
    -e MIN_DRIVER_VERSION=535.129.03
    -e DEBIAN_FRONTEND=noninteractive
    -e LANG=C.UTF-8
    -e LC_ALL=C.UTF-8
    -e PYTHONUNBUFFERED=1
    -e PYTHONDONTWRITEBYTECODE=1
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}"
    -e ROS_LOCALHOST_ONLY=0
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4
    -e SYSTEM_ROS_DISTRO=humble
    -e ISAACSIM_INTERNAL_ROS_DISTRO=jazzy
    -e K1_NAMESPACE="${K1_NAMESPACE}"
    -e K1_POLICY_CONTAINER_PATH="${K1_POLICY_CONTAINER_PATH}"
    -e K1_SCENE_USD="${K1_SCENE_USD}"
    -e K1_CONTROL_PERIOD_MS="${K1_CONTROL_PERIOD_MS}"
    -e K1_POLICY_DECIMATION="${K1_POLICY_DECIMATION}"
    -e K1_POLICY_OBS_DIM="${K1_POLICY_OBS_DIM}"
    -e K1_POLICY_HEIGHT_SCAN_DIM="${K1_POLICY_HEIGHT_SCAN_DIM}"
    -e K1_POLICY_OBS_MODE="${K1_POLICY_OBS_MODE}"
    -e K1_POLICY_COMMAND_SCALE="${K1_POLICY_COMMAND_SCALE}"
    -e K1_POLICY_COMMAND_OFFSET="${K1_POLICY_COMMAND_OFFSET}"
    -e K1_POLICY_ACTION_GAIN="${K1_POLICY_ACTION_GAIN}"
    -e K1_MANUAL_STANDING_RESET="${K1_MANUAL_STANDING_RESET}"
    -e K1_REMAP_POLICY_ACTIONS="${K1_REMAP_POLICY_ACTIONS}"
    -e K1_ROBOT_Z="${K1_ROBOT_Z}"
    -e K1_ROBOT_ASSET_MODE="${K1_ROBOT_ASSET_MODE}"
    -e K1_CONTROLLER_MODE="${K1_CONTROLLER_MODE}"
    -e K1_KINEMATIC_DT_MODE="${K1_KINEMATIC_DT_MODE}"
    -e K1_KINEMATIC_MAX_DT="${K1_KINEMATIC_MAX_DT}"
    -e K1_KINEMATIC_SPEED_SCALE="${K1_KINEMATIC_SPEED_SCALE}"
    -e K1_KINEMATIC_LIN_X_MIN="${K1_KINEMATIC_LIN_X_MIN}"
    -e K1_KINEMATIC_LIN_X_MAX="${K1_KINEMATIC_LIN_X_MAX}"
    -e K1_VIEW_CAMERA_MODE="${K1_VIEW_CAMERA_MODE}"
    -e K1_VIEW_CAMERA_EYE="${K1_VIEW_CAMERA_EYE}"
    -e K1_VIEW_CAMERA_TARGET="${K1_VIEW_CAMERA_TARGET}"
    -e K1_VIEW_CAMERA_FOCAL_LENGTH="${K1_VIEW_CAMERA_FOCAL_LENGTH}"
    -e K1_PORTED_RUNTIME_ROOT="${K1_PORTED_RUNTIME_CONTAINER_ROOT}"
    -e K1_DISABLE_SCENE_REFERENCE="${K1_DISABLE_SCENE_REFERENCE:-0}"
    -e K1_FORCE_ZERO_ACTIONS="${K1_FORCE_ZERO_ACTIONS:-0}"
    -e ISAAC_LIVESTREAM_PORT=49100
    -e ROSCLAW_READY_FILE=/tmp/rosclaw-k1-ready.json
    -e K1_STATE_FILE=/tmp/rosclaw-k1-state.json
    -e PUBLIC_IP="${PUBLIC_IP}"
    -e LIVESTREAM="${LIVESTREAM}"
    -v "${DATA_DIR}/cache/kit:/isaac-sim/kit/cache:rw"
    -v "${DATA_DIR}/cache/ov:/root/.cache/ov:rw"
    -v "${DATA_DIR}/cache/pip:/root/.cache/pip:rw"
    -v "${DATA_DIR}/cache/glcache:/root/.cache/nvidia/GLCache:rw"
    -v "${DATA_DIR}/cache/computecache:/root/.nv/ComputeCache:rw"
    -v "${DATA_DIR}/logs:/root/.nvidia-omniverse/logs:rw"
    -v "${DATA_DIR}/data:/root/.local/share/ov/data:rw"
    -v "${DATA_DIR}/documents:/root/Documents:rw"
)

if [[ -f "${K1_BOOSTER_ASSET_OVERRIDE}" ]]; then
    docker_args+=(
        -v "${K1_BOOSTER_ASSET_OVERRIDE}:/workspace/rosclaw-example-booster-k1/simulators/isaac-sim/src/booster_k1_sim/assets/booster.py:ro"
    )
fi

if [[ -f "${K1_APP_OVERRIDE}" ]]; then
    docker_args+=(
        -v "${K1_APP_OVERRIDE}:/workspace/rosclaw-example-booster-k1/simulators/isaac-sim/app/k1_warehouse_demo.py:ro"
    )
fi

if [[ -f "${K1_ENTRYPOINT_OVERRIDE}" ]]; then
    docker_args+=(
        -v "${K1_ENTRYPOINT_OVERRIDE}:/workspace/rosclaw-example-booster-k1/simulators/isaac-sim/docker/entrypoint-k1.sh:ro"
    )
fi

if [[ -n "${K1_POLICY_HOST_PATH}" ]]; then
    if [[ ! -f "${K1_POLICY_HOST_PATH}" ]]; then
        echo "[run_k1_isaac_sim] policy host path not found: ${K1_POLICY_HOST_PATH}" >&2
        exit 1
    fi
    docker_args+=(
        -v "${K1_POLICY_HOST_PATH}:${K1_POLICY_CONTAINER_PATH}:ro"
    )
fi

docker_args+=(
    -v "${K1_PORTED_RUNTIME_HOST_ROOT}:${K1_PORTED_RUNTIME_CONTAINER_ROOT}:ro"
)

if [[ "${MODE}" == "gui" ]]; then
    if [[ -z "${DISPLAY:-}" ]]; then
        echo "[run_k1_isaac_sim] DISPLAY must be set for GUI mode" >&2
        exit 1
    fi

    gui_command=$'set -euo pipefail\n'
    gui_command+=$'source_if_present() {\n'
    gui_command+=$'    local file="$1"\n'
    gui_command+=$'    if [[ -f "${file}" ]]; then\n'
    gui_command+=$'        # shellcheck source=/dev/null\n'
    gui_command+=$'        source "${file}"\n'
    gui_command+=$'    fi\n'
    gui_command+=$'}\n'
    gui_command+=$'detect_isaac_ros_root() {\n'
    gui_command+=$'    local candidate\n'
    gui_command+=$'    for candidate in \\\n'
    gui_command+=$'        "/isaac-sim/exts/isaacsim.ros2.bridge/${SYSTEM_ROS_DISTRO:-humble}" \\\n'
    gui_command+=$'        "/isaac-sim/exts/isaacsim.ros2.bridge/${ISAACSIM_INTERNAL_ROS_DISTRO:-jazzy}" \\\n'
    gui_command+=$'        "/isaac-sim/exts/isaacsim.ros2.bridge/humble" \\\n'
    gui_command+=$'        "/isaac-sim/exts/isaacsim.ros2.bridge/jazzy"; do\n'
    gui_command+=$'        if [[ -d "${candidate}/rclpy" && -d "${candidate}/lib" ]]; then\n'
    gui_command+=$'            printf "%s" "${candidate}"\n'
    gui_command+=$'            return 0\n'
    gui_command+=$'        fi\n'
    gui_command+=$'    done\n'
    gui_command+=$'    return 1\n'
    gui_command+=$'}\n'
    gui_command+=$'isaac_ros_root="$(detect_isaac_ros_root || true)"\n'
    gui_command+=$'if [[ -n "${isaac_ros_root}" ]]; then\n'
    gui_command+=$'    export ROS_DISTRO="${SYSTEM_ROS_DISTRO:-humble}"\n'
    gui_command+=$'    export PYTHONPATH="${isaac_ros_root}/rclpy${PYTHONPATH:+:${PYTHONPATH}}"\n'
    gui_command+=$'    export LD_LIBRARY_PATH="${isaac_ros_root}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"\n'
    gui_command+=$'fi\n'
    gui_command+=$'/isaac-sim/python.sh /workspace/rosclaw-example-booster-k1/simulators/isaac-sim/app/k1_warehouse_demo.py '
    gui_command+=$'--namespace "${K1_NAMESPACE}" '
    gui_command+=$'--scene-usd "${K1_SCENE_USD}" '
    gui_command+=$'--policy-path "${K1_POLICY_CONTAINER_PATH}" '
    gui_command+=$'--publish-period-ms "${K1_CONTROL_PERIOD_MS}" '
    gui_command+=$'--policy-decimation "${K1_POLICY_DECIMATION}" '
    gui_command+=$'--obs-dim "${K1_POLICY_OBS_DIM}" '
    gui_command+=$'--height-scan-dim "${K1_POLICY_HEIGHT_SCAN_DIM}" '
    gui_command+=$'--robot-z "${K1_ROBOT_Z}" '
    gui_command+=$'--livestream 0 &\n'
    gui_command+=$'isaac_pid=$!\n'
    gui_command+=$'for _ in $(seq 1 180); do\n'
    gui_command+=$'    if [[ -f "${ROSCLAW_READY_FILE:-/tmp/rosclaw-k1-ready.json}" ]]; then\n'
    gui_command+=$'        break\n'
    gui_command+=$'    fi\n'
    gui_command+=$'    if ! kill -0 "${isaac_pid}" 2>/dev/null; then\n'
    gui_command+=$'        wait "${isaac_pid}"\n'
    gui_command+=$'        exit $?\n'
    gui_command+=$'    fi\n'
    gui_command+=$'    sleep 1\n'
    gui_command+=$'done\n'
    gui_command+=$'set +u\n'
    gui_command+=$'source_if_present "/opt/ros/${SYSTEM_ROS_DISTRO:-humble}/setup.bash"\n'
    gui_command+=$'source_if_present "/workspace/booster_sdk_ws/install/setup.bash"\n'
    gui_command+=$'source_if_present "/workspace/rosclaw_ws/install/setup.bash"\n'
    gui_command+=$'set -u\n'
    gui_command+=$'ros2 launch rosclaw_bringup k1.launch.py rosbridge:=true perception:=false use_sim_time:=true &\n'
    gui_command+=$'rosclaw_pid=$!\n'
    gui_command+=$'cleanup() {\n'
    gui_command+=$'    kill "${isaac_pid}" "${rosclaw_pid}" 2>/dev/null || true\n'
    gui_command+=$'    wait "${rosclaw_pid}" 2>/dev/null || true\n'
    gui_command+=$'    wait "${isaac_pid}" 2>/dev/null || true\n'
    gui_command+=$'}\n'
    gui_command+=$'trap cleanup EXIT INT TERM\n'
    gui_command+=$'wait -n "${isaac_pid}" "${rosclaw_pid}"\n'

    exec docker "${docker_args[@]}" \
        -e DISPLAY="${DISPLAY}" \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        --entrypoint /bin/bash \
        "${IMAGE}" -lc "${gui_command}"
fi

if [[ -f "${K1_ENTRYPOINT_OVERRIDE}" ]]; then
    exec docker "${docker_args[@]}" \
        --entrypoint /workspace/rosclaw-example-booster-k1/simulators/isaac-sim/docker/entrypoint-k1.sh \
        "${IMAGE}"
fi

exec docker "${docker_args[@]}" "${IMAGE}"
