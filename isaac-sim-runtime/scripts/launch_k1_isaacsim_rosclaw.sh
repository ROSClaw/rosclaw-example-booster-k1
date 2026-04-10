#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNTIME_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${RUNTIME_ROOT}/.." && pwd)"
ROSCLAW_WS="${ROSCLAW_WS:-${REPO_ROOT}/external/rosclaw-ros2}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
ROSCLAW_SETUP="${ROSCLAW_SETUP:-${ROSCLAW_WS}/install/setup.bash}"
ISAAC_ENV_SCRIPT="${ISAAC_ENV_SCRIPT:-}"
ISAAC_SIM_PYTHON="${ISAAC_SIM_PYTHON:-python3}"

K1_NAMESPACE="${K1_NAMESPACE:-/k1}"
K1_CONTROL_MODE="${K1_CONTROL_MODE:-cmd_vel}"
K1_CONTROLLER_BACKEND="${K1_CONTROLLER_BACKEND:-vendor_cmd_vel}"
K1_TELEOP_DEVICE="${K1_TELEOP_DEVICE:-cmd_vel}"
K1_PERCEPTION="${K1_PERCEPTION:-false}"
K1_USE_SIM_TIME="${K1_USE_SIM_TIME:-true}"
K1_ROSBRIDGE="${K1_ROSBRIDGE:-true}"
K1_REUSE_EXISTING_ROSBRIDGE="${K1_REUSE_EXISTING_ROSBRIDGE:-false}"
K1_WEBRTC="${K1_WEBRTC:-false}"
K1_RELAY="${K1_RELAY:-false}"
K1_ROSBRIDGE_HOST="${K1_ROSBRIDGE_HOST:-127.0.0.1}"
K1_ROSBRIDGE_PORT="${K1_ROSBRIDGE_PORT:-9090}"
K1_POLICY_CHECKPOINT="${K1_POLICY_CHECKPOINT:-}"
K1_MOTION_FILE="${K1_MOTION_FILE:-}"
K1_SCENE_USD="${K1_SCENE_USD:-/Isaac/Environments/Simple_Warehouse/full_warehouse.usd}"
K1_SCENE_PRIM_PATH="${K1_SCENE_PRIM_PATH:-/World/Warehouse}"
K1_PHYSICS_HZ="${K1_PHYSICS_HZ:-200.0}"
K1_RENDER_HZ="${K1_RENDER_HZ:-50.0}"
K1_BROADPHASE_TYPE="${K1_BROADPHASE_TYPE:-MBP}"
K1_ENABLE_GPU_DYNAMICS="${K1_ENABLE_GPU_DYNAMICS:-false}"
K1_SDK_COMPAT="${K1_SDK_COMPAT:-false}"
K1_PLAIN_HEAD="${K1_PLAIN_HEAD:-false}"
GR00T_WBC_ROOT="${GR00T_WBC_ROOT:-}"
K1_GR00T_STRICT="${K1_GR00T_STRICT:-false}"
K1_READY_TOPIC="${K1_READY_TOPIC:-joint_states}"
K1_READY_TIMEOUT_SEC="${K1_READY_TIMEOUT_SEC:-120}"

SIM_NAMESPACE="${K1_NAMESPACE#/}"
SIM_NAMESPACE="${SIM_NAMESPACE%/}"
READY_TOPIC_SUFFIX="${K1_READY_TOPIC#/}"
READY_TOPIC="/${SIM_NAMESPACE:+${SIM_NAMESPACE}/}${READY_TOPIC_SUFFIX}"

LOG_DIR="${RUNTIME_ROOT}/logs"
SIM_LOG="${LOG_DIR}/k1_isaacsim.log"

mkdir -p "${LOG_DIR}"

if [[ -n "${ISAAC_ENV_SCRIPT}" ]]; then
  # shellcheck disable=SC1090
  source "${ISAAC_ENV_SCRIPT}"
fi

if [[ -f "${ROS_SETUP}" ]]; then
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
fi

if [[ ! -f "${ROSCLAW_SETUP}" ]]; then
  echo "[launch] missing ROSClaw install setup: ${ROSCLAW_SETUP}" >&2
  exit 1
fi

# shellcheck disable=SC1090
source "${ROSCLAW_SETUP}"

export PYTHONPATH="${RUNTIME_ROOT}/vendor/booster_assets/src:${RUNTIME_ROOT}/vendor/booster_train/source/booster_train${PYTHONPATH:+:${PYTHONPATH}}"

if [[ -n "${GR00T_WBC_ROOT}" ]]; then
  export GR00T_WBC_ROOT
  export PYTHONPATH="${GR00T_WBC_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
fi

SIM_ARGS=(
  "${RUNTIME_ROOT}/scripts/k1_demo.py"
  "--namespace" "${SIM_NAMESPACE}"
  "--control-mode" "${K1_CONTROL_MODE}"
  "--teleop-device" "${K1_TELEOP_DEVICE}"
  "--controller-backend" "${K1_CONTROLLER_BACKEND}"
  "--scene-usd" "${K1_SCENE_USD}"
  "--scene-prim-path" "${K1_SCENE_PRIM_PATH}"
  "--physics-hz" "${K1_PHYSICS_HZ}"
  "--render-hz" "${K1_RENDER_HZ}"
  "--broadphase-type" "${K1_BROADPHASE_TYPE}"
)

if [[ -n "${K1_POLICY_CHECKPOINT}" ]]; then
  SIM_ARGS+=("--policy-checkpoint" "${K1_POLICY_CHECKPOINT}")
fi

if [[ -n "${K1_MOTION_FILE}" ]]; then
  SIM_ARGS+=("--motion-file" "${K1_MOTION_FILE}")
fi

if [[ "${K1_ENABLE_GPU_DYNAMICS}" == "true" ]]; then
  SIM_ARGS+=("--enable-gpu-dynamics")
fi

if [[ "${K1_SDK_COMPAT}" == "true" ]]; then
  SIM_ARGS+=("--sdk-compat")
fi

if [[ "${K1_PLAIN_HEAD}" == "true" ]]; then
  SIM_ARGS+=("--plain-head")
fi

if [[ -n "${GR00T_WBC_ROOT}" ]]; then
  SIM_ARGS+=("--gr00t-root" "${GR00T_WBC_ROOT}")
fi

if [[ "${K1_GR00T_STRICT}" == "true" ]]; then
  SIM_ARGS+=("--gr00t-strict")
fi

SIM_PID=""
ROSCLAW_PID=""

cleanup() {
  local exit_code=$?
  if [[ -n "${ROSCLAW_PID}" ]] && kill -0 "${ROSCLAW_PID}" 2>/dev/null; then
    kill "${ROSCLAW_PID}" 2>/dev/null || true
    wait "${ROSCLAW_PID}" 2>/dev/null || true
  fi
  if [[ -n "${SIM_PID}" ]] && kill -0 "${SIM_PID}" 2>/dev/null; then
    kill "${SIM_PID}" 2>/dev/null || true
    wait "${SIM_PID}" 2>/dev/null || true
  fi
  exit "${exit_code}"
}

trap cleanup EXIT INT TERM

echo "[launch] starting K1 Isaac Sim runtime; log: ${SIM_LOG}"
"${ISAAC_SIM_PYTHON}" "${SIM_ARGS[@]}" >"${SIM_LOG}" 2>&1 &
SIM_PID=$!

wait_for_ready_topic() {
  local deadline=$((SECONDS + K1_READY_TIMEOUT_SEC))
  while (( SECONDS < deadline )); do
    if ! kill -0 "${SIM_PID}" 2>/dev/null; then
      echo "[launch] K1 Isaac Sim exited before publishing ${READY_TOPIC}" >&2
      wait "${SIM_PID}" || true
      return 1
    fi
    if ros2 topic list 2>/dev/null | grep -Fxq "${READY_TOPIC}"; then
      echo "[launch] simulator ready on ${READY_TOPIC}"
      return 0
    fi
    sleep 1
  done
  echo "[launch] timed out waiting for ${READY_TOPIC}. Check ${SIM_LOG}" >&2
  return 1
}

wait_for_ready_topic

echo "[launch] starting ROSClaw bringup for platform k1"
ros2 launch rosclaw_bringup k1.launch.py \
  robot_namespace:="${K1_NAMESPACE}" \
  use_sim_time:="${K1_USE_SIM_TIME}" \
  perception:="${K1_PERCEPTION}" \
  rosbridge:="${K1_ROSBRIDGE}" \
  reuse_existing_rosbridge:="${K1_REUSE_EXISTING_ROSBRIDGE}" \
  rosbridge_host:="${K1_ROSBRIDGE_HOST}" \
  rosbridge_port:="${K1_ROSBRIDGE_PORT}" \
  webrtc:="${K1_WEBRTC}" \
  relay:="${K1_RELAY}" &
ROSCLAW_PID=$!

wait "${ROSCLAW_PID}"
