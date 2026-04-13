#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOCAL_DDS_SETUP="${REPO_ROOT}/setup-dds-env.sh"
ROSCLAW_SETUP_BASH_DEFAULT="${HOME}/ros2_ws/install/local_setup.bash"

ROBOT_HOST="${K1_ROBOT_HOST:-booster@192.168.2.152}"
ROBOT_RELAY_WS="${K1_ROBOT_RELAY_WS:-/tmp/k1-relay-ws}"
ROSCLAW_SETUP_BASH="${ROSCLAW_SETUP_BASH:-${ROSCLAW_SETUP_BASH_DEFAULT}}"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/roslogs}"
PLATFORM="k1"
ENABLE_ROSBRIDGE="true"
ENABLE_PERCEPTION="false"
ENABLE_ROSCLAW_PACKAGES="true"
ENABLE_AUTONOMY="true"
ENSURE_ROBOT_RELAY="true"
ENABLE_CMD_VEL_BRIDGE="true"
ENABLE_VISIONOS_BACKEND="true"
RELAY_ONLY="false"
FORCE_ROBOT_RELAY_BUILD="false"
DRY_RUN="false"
CMD_VEL_TOPIC="/cmd_vel"
BRIDGE_RPC_SERVICE_NAME="/booster_rpc_service"
AUTONOMY_STARTUP_MODE="${AUTONOMY_STARTUP_MODE:-MANUAL}"
AUTONOMY_DEFAULT_EXECUTOR="${AUTONOMY_DEFAULT_EXECUTOR:-auto}"
AUTONOMY_IDLE_ROAM_STRATEGY="${AUTONOMY_IDLE_ROAM_STRATEGY:-frontier}"
AUTONOMY_START_DELAY_SEC="${AUTONOMY_START_DELAY_SEC:-3}"
VISIONOS_BACKEND_PORT="${VISIONOS_BACKEND_PORT:-8088}"
VISIONOS_OPENCLAW_AGENT_ID="${VISIONOS_OPENCLAW_AGENT_ID:-main}"
VISIONOS_OPENCLAW_SESSION_ID="${VISIONOS_OPENCLAW_SESSION_ID:-${VISIONOS_OPENCLAW_SESSION_KEY:-k1-visionos}}"
ROSCLAW_REPO_CONFIG_ROOT_DEFAULT="${SCRIPT_DIR}/src/k1_cmd_vel_bridge/config"

EXTRA_LAUNCH_ARGS=()

usage() {
  cat <<EOF
Usage: $(basename "$0") [options] [-- extra ros2 launch args]

Bring up the K1 host-side ROS stack for OpenClaw:
  1. Source the project Fast DDS environment.
  2. Optionally deploy/build/launch the robot-local k1 relay over SSH.
  3. Launch ROSClaw bringup from an existing rosclaw-ros2 overlay.

Options:
  --robot HOST              SSH target for the K1 relay deployment.
                            Default: ${ROBOT_HOST}
  --robot-relay-ws PATH     Remote workspace used for the relay.
                            Default: ${ROBOT_RELAY_WS}
  --rosclaw-setup PATH      setup.bash for the built rosclaw-ros2 overlay.
                            Default: ${ROSCLAW_SETUP_BASH}
  --platform NAME           ROSClaw platform argument.
                            Default: ${PLATFORM}
  --no-rosclaw-packages     Skip launching rosclaw_bringup from the overlay.
  --no-autonomy            Skip launching rosclaw_autonomy after rosclaw_bringup.
  --autonomy-startup-mode MODE
                            rosclaw_autonomy startup mode.
                            Default: ${AUTONOMY_STARTUP_MODE}
  --no-rosbridge            Disable rosbridge in rosclaw_bringup.
  --perception              Enable rosclaw perception.
                            Note: K1 currently exposes /image_left_raw, not a
                            CompressedImage topic, so perception likely needs
                            extra launch overrides.
  --no-robot-relay          Skip the remote relay deployment/launch step.
  --no-cmd-vel-bridge       Skip the local /cmd_vel -> Booster RPC bridge.
  --no-visionos-backend     Skip launching the visionOS HTTP/OpenClaw backend.
  --cmd-vel-topic TOPIC     Twist topic to bridge.
                            Default: ${CMD_VEL_TOPIC}
  --rpc-service-name NAME   Booster locomotion RPC service name.
                            Default: ${BRIDGE_RPC_SERVICE_NAME}
  --visionos-backend-port PORT
                            HTTP port for the visionOS backend.
                            Default: ${VISIONOS_BACKEND_PORT}
  Environment override: VISIONOS_OPENCLAW_SESSION_ID
                        Falls back to legacy VISIONOS_OPENCLAW_SESSION_KEY.
  --force-robot-relay-build Re-copy and rebuild the relay workspace on robot.
  --relay-only              Only ensure the robot relay is running.
  --dry-run                 Print commands without executing them.
  -h, --help                Show this help.

Extra args after -- are passed directly to:
  ros2 launch rosclaw_bringup rosclaw.launch.py ...

Examples:
  $(basename "$0")
  $(basename "$0") --no-robot-relay
  $(basename "$0") -- discovery_config:=/path/to/discovery.yaml
EOF
}

log() {
  printf '[bringup] %s\n' "$*"
}

die() {
  printf '[bringup] ERROR: %s\n' "$*" >&2
  exit 1
}

run_cmd() {
  if [[ "${DRY_RUN}" == "true" ]]; then
    printf '[dry-run] '
    printf '%q ' "$@"
    printf '\n'
    return 0
  fi
  "$@"
}

source_compat() {
  local target="$1"
  local restore_nounset=0
  if [[ $- == *u* ]]; then
    restore_nounset=1
    set +u
  fi
  # shellcheck disable=SC1090
  source "${target}"
  if [[ "${restore_nounset}" == "1" ]]; then
    set -u
  fi
}

reset_ros_overlay_env() {
  unset AMENT_PREFIX_PATH
  unset CMAKE_PREFIX_PATH
  unset COLCON_PREFIX_PATH
  unset LD_LIBRARY_PATH
  unset PYTHONPATH
}

launch_arg_override_present() {
  local arg_name="$1"
  local entry
  for entry in "${EXTRA_LAUNCH_ARGS[@]}"; do
    if [[ "${entry}" == "${arg_name}:="* ]]; then
      return 0
    fi
  done
  return 1
}

append_repo_platform_config_override() {
  local arg_name="$1"
  local filename="$2"
  local config_path="${ROSCLAW_REPO_CONFIG_ROOT_DEFAULT}/${PLATFORM}/${filename}"
  if launch_arg_override_present "${arg_name}"; then
    return 0
  fi
  if [[ -f "${config_path}" ]]; then
    EXTRA_LAUNCH_ARGS+=("${arg_name}:=${config_path}")
  fi
}

run_remote_script() {
  local script_body="$1"
  shift
  if [[ "${DRY_RUN}" == "true" ]]; then
    printf '[dry-run] ssh %q bash -s --' "${ROBOT_HOST}"
    for arg in "$@"; do
      printf ' %q' "${arg}"
    done
    printf ' <<REMOTE_SCRIPT\n%s\nREMOTE_SCRIPT\n' "${script_body}"
    return 0
  fi
  ssh "${SSH_OPTS[@]}" "${ROBOT_HOST}" bash -s -- "$@" <<<"${script_body}"
}

copy_to_robot() {
  local source_path="$1"
  local dest_path="$2"
  run_cmd scp "${SCP_OPTS[@]}" -r "${source_path}" "${ROBOT_HOST}:${dest_path}"
}

while (($# > 0)); do
  case "$1" in
    --robot)
      ROBOT_HOST="$2"
      shift 2
      ;;
    --robot-relay-ws)
      ROBOT_RELAY_WS="$2"
      shift 2
      ;;
    --rosclaw-setup)
      ROSCLAW_SETUP_BASH="$2"
      shift 2
      ;;
    --platform)
      PLATFORM="$2"
      shift 2
      ;;
    --no-rosclaw-packages)
      ENABLE_ROSCLAW_PACKAGES="false"
      shift
      ;;
    --no-autonomy)
      ENABLE_AUTONOMY="false"
      shift
      ;;
    --autonomy-startup-mode)
      AUTONOMY_STARTUP_MODE="$2"
      shift 2
      ;;
    --no-rosbridge)
      ENABLE_ROSBRIDGE="false"
      shift
      ;;
    --perception)
      ENABLE_PERCEPTION="true"
      shift
      ;;
    --no-robot-relay)
      ENSURE_ROBOT_RELAY="false"
      shift
      ;;
    --no-cmd-vel-bridge)
      ENABLE_CMD_VEL_BRIDGE="false"
      shift
      ;;
    --no-visionos-backend)
      ENABLE_VISIONOS_BACKEND="false"
      shift
      ;;
    --cmd-vel-topic)
      CMD_VEL_TOPIC="$2"
      shift 2
      ;;
    --rpc-service-name)
      BRIDGE_RPC_SERVICE_NAME="$2"
      shift 2
      ;;
    --visionos-backend-port)
      VISIONOS_BACKEND_PORT="$2"
      shift 2
      ;;
    --force-robot-relay-build)
      FORCE_ROBOT_RELAY_BUILD="true"
      shift
      ;;
    --relay-only)
      RELAY_ONLY="true"
      shift
      ;;
    --dry-run)
      DRY_RUN="true"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      for extra_arg in "$@"; do
        if [[ "${extra_arg}" == --*:=* ]]; then
          EXTRA_LAUNCH_ARGS+=("${extra_arg#--}")
        else
          EXTRA_LAUNCH_ARGS+=("${extra_arg}")
        fi
      done
      break
      ;;
    *)
      die "Unknown argument: $1"
      ;;
  esac
done

[[ -f "${LOCAL_DDS_SETUP}" ]] || die "Missing DDS setup script: ${LOCAL_DDS_SETUP}"

RUN_LOCAL_HOST_STACK="false"
if [[ "${ENABLE_ROSCLAW_PACKAGES}" == "true" || "${ENABLE_CMD_VEL_BRIDGE}" == "true" ]]; then
  RUN_LOCAL_HOST_STACK="true"
fi
if [[ "${ENABLE_VISIONOS_BACKEND}" == "true" ]]; then
  RUN_LOCAL_HOST_STACK="true"
fi

if [[ "${RELAY_ONLY}" != "true" && "${RUN_LOCAL_HOST_STACK}" == "true" ]]; then
  [[ -f "${ROSCLAW_SETUP_BASH}" ]] || die "Missing ROSClaw overlay setup: ${ROSCLAW_SETUP_BASH}"
fi

append_repo_platform_config_override "discovery_config" "discovery.yaml"
append_repo_platform_config_override "safety_config" "safety_policy.yaml"
if [[ "${ENABLE_PERCEPTION}" == "true" ]]; then
  append_repo_platform_config_override "perception_config" "perception.yaml"
fi

mkdir -p "${ROS_LOG_DIR}"

SSH_OPTS=(-o StrictHostKeyChecking=no)
SCP_OPTS=(-o StrictHostKeyChecking=no)

ensure_robot_relay() {
  local remote_check_script remote_prep_script remote_build_script remote_launch_script

  log "Ensuring robot relay on ${ROBOT_HOST}"

  if [[ "${FORCE_ROBOT_RELAY_BUILD}" != "true" ]]; then
    remote_check_script=$(cat <<'EOF'
set -euo pipefail
robot_ws="$1"
node_exe="${robot_ws}/install/k1_low_level_relay/lib/k1_low_level_relay/k1_low_level_relay_node"
if [[ -x "${node_exe}" ]] && pgrep -f "${node_exe}" >/dev/null 2>&1; then
  exit 0
fi
exit 1
EOF
)
    if run_remote_script "${remote_check_script}" "${ROBOT_RELAY_WS}"; then
      log "Robot relay is already running; skipping deploy/build"
      return 0
    fi
  fi

  remote_check_script=$(cat <<'EOF'
set -euo pipefail
robot_ws="$1"
if [[ -f "${robot_ws}/install/setup.bash" ]]; then
  exit 0
fi
exit 1
EOF
)

  if [[ "${FORCE_ROBOT_RELAY_BUILD}" == "true" ]] || ! run_remote_script "${remote_check_script}" "${ROBOT_RELAY_WS}"; then
    log "Preparing relay workspace on robot at ${ROBOT_RELAY_WS}"
    remote_prep_script=$(cat <<'EOF'
set -euo pipefail
robot_ws="$1"
mkdir -p "${robot_ws}/src"
rm -rf "${robot_ws}/src/k1_low_level_relay"
rm -rf "${robot_ws}/src/booster_ros2_interface"
rm -rf "${robot_ws}/src/.workflow"
EOF
)
    run_remote_script "${remote_prep_script}" "${ROBOT_RELAY_WS}"

    log "Copying relay sources to robot"
    copy_to_robot "${SCRIPT_DIR}/src/k1_low_level_relay" "${ROBOT_RELAY_WS}/src/"
    copy_to_robot "${SCRIPT_DIR}/src/booster_robotics_sdk_ros2/booster_ros2_interface" "${ROBOT_RELAY_WS}/src/"
    copy_to_robot "${SCRIPT_DIR}/src/booster_robotics_sdk_ros2/.workflow" "${ROBOT_RELAY_WS}/src/"

    log "Building relay workspace on robot"
    remote_build_script=$(cat <<'EOF'
set -eo pipefail
robot_ws="$1"
set +u
source /opt/ros/humble/setup.bash >/dev/null 2>&1
set -u
cd "${robot_ws}"
colcon build \
  --packages-select booster_interface k1_low_level_relay \
  --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
EOF
)
    run_remote_script "${remote_build_script}" "${ROBOT_RELAY_WS}"
  else
    log "Robot relay workspace already exists; skipping rebuild"
  fi

  log "Launching robot relay"
  remote_launch_script=$(cat <<'EOF'
set -eo pipefail
robot_ws="$1"
node_exe="${robot_ws}/install/k1_low_level_relay/lib/k1_low_level_relay/k1_low_level_relay_node"
pkill -f "${node_exe}" >/dev/null 2>&1 || true
set +u
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source "${robot_ws}/install/setup.bash"
set -u
unset FASTDDS_DEFAULT_PROFILES_FILE RMW_IMPLEMENTATION
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile.xml
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_LOG_DIR=/tmp/roslogs
mkdir -p "${ROS_LOG_DIR}"
nohup "${node_exe}" --ros-args -r __node:=k1_low_level_relay >/tmp/k1-relay.log 2>&1 < /dev/null &
sleep 3
pgrep -af "${node_exe}"
EOF
)
  run_remote_script "${remote_launch_script}" "${ROBOT_RELAY_WS}"
}

if [[ "${ENSURE_ROBOT_RELAY}" == "true" ]]; then
  ensure_robot_relay
fi

if [[ "${RELAY_ONLY}" == "true" ]]; then
  log "Relay-only mode complete"
  exit 0
fi

log "Launching local host stack (rosclaw_packages=${ENABLE_ROSCLAW_PACKAGES}, platform=${PLATFORM}, rosbridge=${ENABLE_ROSBRIDGE}, perception=${ENABLE_PERCEPTION}, cmd_vel_bridge=${ENABLE_CMD_VEL_BRIDGE}, autonomy=${ENABLE_AUTONOMY}, visionos_backend=${ENABLE_VISIONOS_BACKEND})"
if [[ "${DRY_RUN}" != "true" && "${RUN_LOCAL_HOST_STACK}" == "true" ]]; then
  reset_ros_overlay_env
  export ROSCLAW_DDS_SKIP_ROSCLAW_OVERLAY=1
  source_compat "${LOCAL_DDS_SETUP}" >/tmp/rosclaw-dds-env.log
  unset ROSCLAW_DDS_SKIP_ROSCLAW_OVERLAY
  source_compat "${ROSCLAW_SETUP_BASH}"
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  export ROS_LOG_DIR
  log "ROS 2 CLI note: if another shell shows an incomplete topic list, source ${LOCAL_DDS_SETUP} there and run 'ros2 daemon stop' or use '--no-daemon'."
  if [[ "${ENABLE_ROSCLAW_PACKAGES}" == "true" && "${ENABLE_AUTONOMY}" == "true" ]]; then
    if ! ros2 pkg prefix rosclaw_autonomy >/dev/null 2>&1; then
      die "rosclaw_autonomy is not available in the sourced overlays. Build the real-hardware workspace after initializing real-hardware/src/rosclaw-ros2-autonomy."
    fi
  fi
  if [[ "${ENABLE_VISIONOS_BACKEND}" == "true" ]]; then
    if ! ros2 pkg prefix k1_openclaw_mission_bridge >/dev/null 2>&1; then
      die "k1_openclaw_mission_bridge is not available in the sourced overlays. Build the real-hardware workspace."
    fi
    if ! ros2 pkg prefix k1_visionos_rtabmap_bridge >/dev/null 2>&1; then
      die "k1_visionos_rtabmap_bridge is not available in the sourced overlays. Build the real-hardware workspace."
    fi
  fi
fi

LOCAL_LAUNCH_COMMAND=(
  ros2 launch rosclaw_bringup rosclaw.launch.py
  "platform:=${PLATFORM}"
  "rosbridge:=${ENABLE_ROSBRIDGE}"
  "perception:=${ENABLE_PERCEPTION}"
)
LOCAL_LAUNCH_COMMAND+=("${EXTRA_LAUNCH_ARGS[@]}")

BRIDGE_COMMAND=(
  ros2 run k1_cmd_vel_bridge k1_cmd_vel_bridge_node
  --ros-args
  -p "cmd_vel_topic:=${CMD_VEL_TOPIC}"
  -p "rpc_service_name:=${BRIDGE_RPC_SERVICE_NAME}"
)

AUTONOMY_COMMAND=(
  ros2 run rosclaw_autonomy autonomy_node
  --ros-args
  -p "startup_mode:=${AUTONOMY_STARTUP_MODE}"
  -p "default_executor:=${AUTONOMY_DEFAULT_EXECUTOR}"
  -p "idle_roam_strategy:=${AUTONOMY_IDLE_ROAM_STRATEGY}"
  -p "cmd_vel_topic:=${CMD_VEL_TOPIC}"
)

VISIONOS_BACKEND_COMMAND=(
  ros2 launch k1_openclaw_mission_bridge k1_visionos_backend.launch.py
  "port:=${VISIONOS_BACKEND_PORT}"
  "openclaw_agent_id:=${VISIONOS_OPENCLAW_AGENT_ID}"
  "openclaw_session_id:=${VISIONOS_OPENCLAW_SESSION_ID}"
)

cleanup() {
  if [[ -n "${autonomy_pid:-}" ]]; then
    kill "${autonomy_pid}" >/dev/null 2>&1 || true
    wait "${autonomy_pid}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${bridge_pid:-}" ]]; then
    kill "${bridge_pid}" >/dev/null 2>&1 || true
    wait "${bridge_pid}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${visionos_backend_pid:-}" ]]; then
    kill "${visionos_backend_pid}" >/dev/null 2>&1 || true
    wait "${visionos_backend_pid}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${rosclaw_pid:-}" ]]; then
    kill "${rosclaw_pid}" >/dev/null 2>&1 || true
    wait "${rosclaw_pid}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

if [[ "${RUN_LOCAL_HOST_STACK}" != "true" ]]; then
  log "No local host packages requested; nothing else to launch"
  exit 0
fi

if [[ "${DRY_RUN}" == "true" ]]; then
  if [[ "${ENABLE_CMD_VEL_BRIDGE}" == "true" ]]; then
    run_cmd "${BRIDGE_COMMAND[@]}"
  fi
  if [[ "${ENABLE_VISIONOS_BACKEND}" == "true" ]]; then
    run_cmd "${VISIONOS_BACKEND_COMMAND[@]}"
  fi
  if [[ "${ENABLE_ROSCLAW_PACKAGES}" == "true" ]]; then
    run_cmd "${LOCAL_LAUNCH_COMMAND[@]}"
    if [[ "${ENABLE_AUTONOMY}" == "true" ]]; then
      run_cmd "${AUTONOMY_COMMAND[@]}"
    fi
  else
    log "Skipping ROSClaw package launch"
  fi
else
  if [[ "${ENABLE_VISIONOS_BACKEND}" == "true" ]]; then
    "${VISIONOS_BACKEND_COMMAND[@]}" &
    visionos_backend_pid=$!
    sleep 2
  fi
  if [[ "${ENABLE_ROSCLAW_PACKAGES}" == "true" ]]; then
    if [[ "${ENABLE_CMD_VEL_BRIDGE}" == "true" ]]; then
      "${BRIDGE_COMMAND[@]}" &
      bridge_pid=$!
      sleep 2
    fi
    "${LOCAL_LAUNCH_COMMAND[@]}" &
    rosclaw_pid=$!
    if [[ "${ENABLE_AUTONOMY}" == "true" ]]; then
      sleep "${AUTONOMY_START_DELAY_SEC}"
      if kill -0 "${rosclaw_pid}" >/dev/null 2>&1; then
        log "Launching rosclaw_autonomy (startup_mode=${AUTONOMY_STARTUP_MODE})"
        "${AUTONOMY_COMMAND[@]}" &
        autonomy_pid=$!
      else
        log "Skipping rosclaw_autonomy launch because rosclaw_bringup exited early"
      fi
    fi
    wait "${rosclaw_pid}"
  elif [[ "${ENABLE_CMD_VEL_BRIDGE}" == "true" ]]; then
    exec "${BRIDGE_COMMAND[@]}"
  else
    log "Skipping ROSClaw package launch"
  fi
fi
