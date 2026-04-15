#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOCAL_DDS_SETUP="${REPO_ROOT}/setup-dds-env.sh"
ROSCLAW_SETUP_BASH_DEFAULT="${HOME}/ros2_ws/install/local_setup.bash"

ROBOT_HOST="${K1_ROBOT_HOST:-booster@192.168.2.152}"
SSH_PASSWORD="${K1_SSH_PASSWORD:-}"
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
ENABLE_STEREO_CAMERA_BRIDGE="true"
RELAY_ONLY="false"
FORCE_ROBOT_RELAY_BUILD="false"
FORCE_LOCAL_HOST_BUILD="false"
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
STEREO_STREAM_HOST="${K1_STEREO_STREAM_HOST:-}"
STEREO_STREAM_PORT="${K1_STEREO_STREAM_PORT:-5600}"
STEREO_STREAM_OUTPUT_ENCODING="${K1_STEREO_STREAM_OUTPUT_ENCODING:-mono8}"
STEREO_STREAM_OUTPUT_WIDTH="${K1_STEREO_STREAM_OUTPUT_WIDTH:-512}"
STEREO_STREAM_OUTPUT_HEIGHT="${K1_STEREO_STREAM_OUTPUT_HEIGHT:-512}"
STEREO_STREAM_MAX_FPS="${K1_STEREO_STREAM_MAX_FPS:-12.0}"
STEREO_STREAM_JPEG_QUALITY="${K1_STEREO_STREAM_JPEG_QUALITY:-80}"

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
  --ssh-password PASS       Non-interactive SSH password for the K1.
                            Requires sshpass when set.
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
  --no-stereo-camera-bridge Skip launching the stereo camera host bridge.
  --cmd-vel-topic TOPIC     Twist topic to bridge.
                            Default: ${CMD_VEL_TOPIC}
  --rpc-service-name NAME   Booster locomotion RPC service name.
                            Default: ${BRIDGE_RPC_SERVICE_NAME}
  --visionos-backend-port PORT
                            HTTP port for the visionOS backend.
                            Default: ${VISIONOS_BACKEND_PORT}
  --stereo-stream-port PORT TCP port for robot stereo camera transport.
                            Default: ${STEREO_STREAM_PORT}
  Environment override: VISIONOS_OPENCLAW_SESSION_ID
                        Falls back to legacy VISIONOS_OPENCLAW_SESSION_KEY.
  --force-local-host-build
                            Rebuild relevant local host packages in
                            real-hardware before launching.
  --force-robot-relay-build Re-copy and rebuild the relay workspace on robot.
  --relay-only              Only ensure the robot relay is running.
  --dry-run                 Print commands without executing them.
  -h, --help                Show this help.

Environment:
  K1_ROBOT_HOST             Default SSH target for the K1 relay deployment.
  K1_SSH_PASSWORD           Optional non-interactive SSH password.
                            Without it, SSH connection sharing reduces the
                            relay path to a single password prompt.

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

SSH_OPTS=(-o StrictHostKeyChecking=no)
SCP_OPTS=(-o StrictHostKeyChecking=no)
SSH_CMD=(ssh)
SCP_CMD=(scp)
SSH_CONTROL_PATH=""
SSH_MASTER_STARTED="false"

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

configure_ssh_transport() {
  if [[ -z "${SSH_CONTROL_PATH}" ]]; then
    SSH_CONTROL_PATH="${TMPDIR:-/tmp}/rosclaw-k1-ssh-%r@%h:%p"
  fi

  SSH_OPTS=(
    -o StrictHostKeyChecking=no
    -o ControlMaster=auto
    -o ControlPersist=10m
    -o "ControlPath=${SSH_CONTROL_PATH}"
  )
  SCP_OPTS=(
    -o StrictHostKeyChecking=no
    -o ControlMaster=auto
    -o ControlPersist=10m
    -o "ControlPath=${SSH_CONTROL_PATH}"
  )

  SSH_CMD=(ssh)
  SCP_CMD=(scp)
  if [[ -n "${SSH_PASSWORD}" ]]; then
    if [[ "${DRY_RUN}" != "true" ]]; then
      command -v sshpass >/dev/null 2>&1 || die "sshpass is required when --ssh-password or K1_SSH_PASSWORD is set"
    fi
    SSH_CMD=(sshpass -p "${SSH_PASSWORD}" ssh -o PubkeyAuthentication=no -o PreferredAuthentications=password)
    SCP_CMD=(sshpass -p "${SSH_PASSWORD}" scp -o PubkeyAuthentication=no -o PreferredAuthentications=password)
  fi
}

start_ssh_master() {
  [[ "${ENSURE_ROBOT_RELAY}" == "true" ]] || return 0
  [[ "${SSH_MASTER_STARTED}" != "true" ]] || return 0
  configure_ssh_transport

  if [[ "${DRY_RUN}" == "true" ]]; then
    printf '[dry-run]'
    printf ' %q' "${SSH_CMD[@]}" "${SSH_OPTS[@]}" -MNf "${ROBOT_HOST}"
    printf '\n'
    SSH_MASTER_STARTED="true"
    return 0
  fi

  "${SSH_CMD[@]}" "${SSH_OPTS[@]}" -MNf "${ROBOT_HOST}"
  SSH_MASTER_STARTED="true"
}

stop_ssh_master() {
  [[ "${SSH_MASTER_STARTED}" == "true" ]] || return 0
  if [[ "${DRY_RUN}" != "true" ]]; then
    "${SSH_CMD[@]}" "${SSH_OPTS[@]}" -O exit "${ROBOT_HOST}" >/dev/null 2>&1 || true
  fi
  SSH_MASTER_STARTED="false"
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

source_local_host_env() {
  reset_ros_overlay_env
  export ROSCLAW_DDS_SKIP_ROSCLAW_OVERLAY=1
  source_compat "${LOCAL_DDS_SETUP}" >/tmp/rosclaw-dds-env.log
  unset ROSCLAW_DDS_SKIP_ROSCLAW_OVERLAY
  source_compat "${ROSCLAW_SETUP_BASH}"
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  export ROS_LOG_DIR
}

package_available() {
  local package_name="$1"
  ros2 pkg prefix "${package_name}" >/dev/null 2>&1
}

LOCAL_HOST_BUILD_PACKAGES=()

append_local_host_build_package() {
  local package_name="$1"
  local existing_package
  for existing_package in "${LOCAL_HOST_BUILD_PACKAGES[@]:-}"; do
    if [[ "${existing_package}" == "${package_name}" ]]; then
      return 0
    fi
  done
  LOCAL_HOST_BUILD_PACKAGES+=("${package_name}")
}

schedule_local_host_package_group() {
  local group_name="$1"
  shift
  local package_name
  local should_build="${FORCE_LOCAL_HOST_BUILD}"

  if [[ "${should_build}" != "true" ]]; then
    should_build="false"
    for package_name in "$@"; do
      if ! package_available "${package_name}"; then
        should_build="true"
        break
      fi
    done
  fi

  if [[ "${should_build}" != "true" ]]; then
    return 0
  fi

  if [[ "${FORCE_LOCAL_HOST_BUILD}" == "true" ]]; then
    log "Force-building ${group_name} packages in the local host workspace"
  else
    log "${group_name} packages are missing from the sourced overlays; building them in the local host workspace"
  fi

  for package_name in "$@"; do
    append_local_host_build_package "${package_name}"
  done
}

build_local_host_packages() {
  local build_packages=("$@")
  [[ ${#build_packages[@]} -gt 0 ]] || return 0

  log "Building local host packages: ${build_packages[*]}"
  (
    set -eo pipefail
    cd "${SCRIPT_DIR}"
    colcon build --packages-select "${build_packages[@]}"
  )
}

ensure_local_host_packages() {
  LOCAL_HOST_BUILD_PACKAGES=()

  if [[ "${ENABLE_CMD_VEL_BRIDGE}" == "true" ]]; then
    schedule_local_host_package_group "K1 cmd_vel bridge" booster_interface k1_cmd_vel_bridge
  fi

  if [[ "${ENABLE_ROSCLAW_PACKAGES}" == "true" && "${ENABLE_AUTONOMY}" == "true" ]]; then
    schedule_local_host_package_group "ROSClaw autonomy" rosclaw_autonomy_msgs rosclaw_autonomy
  fi

  if [[ "${ENABLE_VISIONOS_BACKEND}" == "true" ]]; then
    schedule_local_host_package_group "visionOS backend" rosclaw_autonomy_msgs k1_openclaw_mission_bridge k1_visionos_rtabmap_bridge
  fi

  if [[ "${ENABLE_STEREO_CAMERA_BRIDGE}" == "true" ]]; then
    schedule_local_host_package_group "stereo camera bridge" k1_low_level_relay
  fi

  if [[ ${#LOCAL_HOST_BUILD_PACKAGES[@]} -gt 0 ]]; then
    build_local_host_packages "${LOCAL_HOST_BUILD_PACKAGES[@]}"
    source_local_host_env
  fi
}

run_remote_script() {
  local script_body="$1"
  shift
  configure_ssh_transport
  if [[ "${DRY_RUN}" == "true" ]]; then
    printf '[dry-run]'
    printf ' %q' "${SSH_CMD[@]}" "${SSH_OPTS[@]}" "${ROBOT_HOST}" bash -s --
    for arg in "$@"; do
      printf ' %q' "${arg}"
    done
    printf ' <<REMOTE_SCRIPT\n%s\nREMOTE_SCRIPT\n' "${script_body}"
    return 0
  fi
  "${SSH_CMD[@]}" "${SSH_OPTS[@]}" "${ROBOT_HOST}" bash -s -- "$@" <<<"${script_body}"
}

copy_to_robot() {
  local source_path="$1"
  local dest_path="$2"
  configure_ssh_transport
  if [[ "${DRY_RUN}" == "true" ]]; then
    printf '[dry-run]'
    printf ' %q' "${SCP_CMD[@]}" "${SCP_OPTS[@]}" -r "${source_path}" "${ROBOT_HOST}:${dest_path}"
    printf '\n'
    return 0
  fi
  "${SCP_CMD[@]}" "${SCP_OPTS[@]}" -r "${source_path}" "${ROBOT_HOST}:${dest_path}"
}

while (($# > 0)); do
  case "$1" in
    --robot)
      ROBOT_HOST="$2"
      shift 2
      ;;
    --ssh-password)
      SSH_PASSWORD="$2"
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
    --no-stereo-camera-bridge)
      ENABLE_STEREO_CAMERA_BRIDGE="false"
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
    --stereo-stream-port)
      STEREO_STREAM_PORT="$2"
      shift 2
      ;;
    --force-robot-relay-build)
      FORCE_ROBOT_RELAY_BUILD="true"
      shift
      ;;
    --force-local-host-build)
      FORCE_LOCAL_HOST_BUILD="true"
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

if [[ -z "${STEREO_STREAM_HOST}" ]]; then
  STEREO_STREAM_HOST="${ROBOT_HOST##*@}"
fi

[[ -f "${LOCAL_DDS_SETUP}" ]] || die "Missing DDS setup script: ${LOCAL_DDS_SETUP}"

RUN_LOCAL_HOST_STACK="false"
if [[ "${ENABLE_ROSCLAW_PACKAGES}" == "true" || "${ENABLE_CMD_VEL_BRIDGE}" == "true" ]]; then
  RUN_LOCAL_HOST_STACK="true"
fi
if [[ "${ENABLE_VISIONOS_BACKEND}" == "true" ]]; then
  RUN_LOCAL_HOST_STACK="true"
fi
if [[ "${ENABLE_STEREO_CAMERA_BRIDGE}" == "true" ]]; then
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
trap stop_ssh_master EXIT

ensure_robot_relay() {
  local remote_check_script remote_prep_script remote_build_script remote_launch_script

  log "Ensuring robot relay on ${ROBOT_HOST}"
  start_ssh_master

  if [[ "${FORCE_ROBOT_RELAY_BUILD}" != "true" ]]; then
    remote_check_script=$(cat <<'EOF'
set -euo pipefail
robot_ws="$1"
low_node_exe="${robot_ws}/install/k1_low_level_relay/lib/k1_low_level_relay/k1_low_level_relay_node"
camera_server_exe="${robot_ws}/install/k1_low_level_relay/lib/k1_low_level_relay/k1_stereo_stream_server_node"
if [[ -x "${low_node_exe}" ]] && [[ -x "${camera_server_exe}" ]] && \
   pgrep -f "${low_node_exe}" >/dev/null 2>&1 && \
   pgrep -f "${camera_server_exe}" >/dev/null 2>&1; then
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
low_node_exe="${robot_ws}/install/k1_low_level_relay/lib/k1_low_level_relay/k1_low_level_relay_node"
camera_server_exe="${robot_ws}/install/k1_low_level_relay/lib/k1_low_level_relay/k1_stereo_stream_server_node"
stream_port="$2"
output_encoding="$3"
output_width="$4"
output_height="$5"
max_fps="$6"
jpeg_quality="$7"
pkill -f "${low_node_exe}" >/dev/null 2>&1 || true
pkill -f "${camera_server_exe}" >/dev/null 2>&1 || true
set +u
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source /opt/booster/BoosterRos2/install/setup.bash >/dev/null 2>&1
source "${robot_ws}/install/setup.bash"
set -u
unset FASTDDS_DEFAULT_PROFILES_FILE RMW_IMPLEMENTATION
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_LOG_DIR=/tmp/roslogs
mkdir -p "${ROS_LOG_DIR}"
nohup "${low_node_exe}" --ros-args -r __node:=k1_low_level_relay >/tmp/k1-relay.log 2>&1 < /dev/null &
nohup "${camera_server_exe}" --ros-args -r __node:=k1_stereo_stream_server \
  -p "bind_host:=0.0.0.0" \
  -p "port:=${stream_port}" \
  -p "output_encoding:=${output_encoding}" \
  -p "output_width:=${output_width}" \
  -p "output_height:=${output_height}" \
  -p "max_fps:=${max_fps}" \
  -p "jpeg_quality:=${jpeg_quality}" >/tmp/k1-camera-stream-server.log 2>&1 < /dev/null &
sleep 3
pgrep -af "${low_node_exe}"
pgrep -af "${camera_server_exe}"
EOF
)
  run_remote_script \
    "${remote_launch_script}" \
    "${ROBOT_RELAY_WS}" \
    "${STEREO_STREAM_PORT}" \
    "${STEREO_STREAM_OUTPUT_ENCODING}" \
    "${STEREO_STREAM_OUTPUT_WIDTH}" \
    "${STEREO_STREAM_OUTPUT_HEIGHT}" \
    "${STEREO_STREAM_MAX_FPS}" \
    "${STEREO_STREAM_JPEG_QUALITY}"
}

if [[ "${ENSURE_ROBOT_RELAY}" == "true" ]]; then
  ensure_robot_relay
fi

if [[ "${RELAY_ONLY}" == "true" ]]; then
  log "Relay-only mode complete"
  exit 0
fi

log "Launching local host stack (rosclaw_packages=${ENABLE_ROSCLAW_PACKAGES}, platform=${PLATFORM}, rosbridge=${ENABLE_ROSBRIDGE}, perception=${ENABLE_PERCEPTION}, cmd_vel_bridge=${ENABLE_CMD_VEL_BRIDGE}, autonomy=${ENABLE_AUTONOMY}, visionos_backend=${ENABLE_VISIONOS_BACKEND}, stereo_camera_bridge=${ENABLE_STEREO_CAMERA_BRIDGE})"
if [[ "${DRY_RUN}" != "true" && "${RUN_LOCAL_HOST_STACK}" == "true" ]]; then
  source_local_host_env
  ensure_local_host_packages
  log "ROS 2 CLI note: if another shell shows an incomplete topic list, source ${LOCAL_DDS_SETUP} there and run 'ros2 daemon stop' or use '--no-daemon'."
  if [[ "${ENABLE_CMD_VEL_BRIDGE}" == "true" ]]; then
    if ! package_available booster_interface; then
      die "booster_interface is not available in the sourced overlays. Build the real-hardware workspace."
    fi
    if ! package_available k1_cmd_vel_bridge; then
      die "k1_cmd_vel_bridge is not available in the sourced overlays. Build the real-hardware workspace."
    fi
  fi
  if [[ "${ENABLE_ROSCLAW_PACKAGES}" == "true" && "${ENABLE_AUTONOMY}" == "true" ]]; then
    if ! package_available rosclaw_autonomy_msgs; then
      die "rosclaw_autonomy_msgs is not available in the sourced overlays. Build the real-hardware workspace after initializing real-hardware/src/rosclaw-ros2-autonomy."
    fi
    if ! ros2 pkg prefix rosclaw_autonomy >/dev/null 2>&1; then
      die "rosclaw_autonomy is not available in the sourced overlays. Build the real-hardware workspace after initializing real-hardware/src/rosclaw-ros2-autonomy."
    fi
  fi
  if [[ "${ENABLE_VISIONOS_BACKEND}" == "true" ]]; then
    if ! package_available rosclaw_autonomy_msgs; then
      die "rosclaw_autonomy_msgs is not available in the sourced overlays. Build the real-hardware workspace."
    fi
    if ! package_available k1_openclaw_mission_bridge; then
      die "k1_openclaw_mission_bridge is not available in the sourced overlays. Build the real-hardware workspace."
    fi
    if ! package_available k1_visionos_rtabmap_bridge; then
      die "k1_visionos_rtabmap_bridge is not available in the sourced overlays. Build the real-hardware workspace."
    fi
  fi
  if [[ "${ENABLE_STEREO_CAMERA_BRIDGE}" == "true" ]]; then
    if ! package_available k1_low_level_relay; then
      die "k1_low_level_relay is not available in the sourced overlays. Build the real-hardware workspace."
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

CAMERA_STREAM_CLIENT_COMMAND=(
  ros2 run k1_low_level_relay k1_stereo_stream_client_node
  --ros-args
  -p "server_host:=${STEREO_STREAM_HOST}"
  -p "server_port:=${STEREO_STREAM_PORT}"
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
  if [[ -n "${camera_stream_client_pid:-}" ]]; then
    kill "${camera_stream_client_pid}" >/dev/null 2>&1 || true
    wait "${camera_stream_client_pid}" >/dev/null 2>&1 || true
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
  if [[ "${ENABLE_STEREO_CAMERA_BRIDGE}" == "true" ]]; then
    run_cmd "${CAMERA_STREAM_CLIENT_COMMAND[@]}"
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
  if [[ "${ENABLE_STEREO_CAMERA_BRIDGE}" == "true" ]]; then
    "${CAMERA_STREAM_CLIENT_COMMAND[@]}" &
    camera_stream_client_pid=$!
    sleep 2
  fi
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
  elif [[ -n "${camera_stream_client_pid:-}" || -n "${visionos_backend_pid:-}" ]]; then
    auxiliary_pids=()
    if [[ -n "${camera_stream_client_pid:-}" ]]; then
      auxiliary_pids+=("${camera_stream_client_pid}")
    fi
    if [[ -n "${visionos_backend_pid:-}" ]]; then
      auxiliary_pids+=("${visionos_backend_pid}")
    fi
    log "Waiting on auxiliary host processes"
    wait "${auxiliary_pids[@]}"
  else
    log "Skipping ROSClaw package launch"
  fi
fi
