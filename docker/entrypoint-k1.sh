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
    if [[ -f "/workspace/rosclaw_ws/install/setup.bash" ]]; then
        # shellcheck source=/dev/null
        source "/workspace/rosclaw_ws/install/setup.bash"
    fi
    set -u
}

disable_policy_checkpoint() {
    export K1_POLICY_FALLBACK_USED="0"
    export K1_ACTIVE_POLICY_CHECKPOINT=""
    export K1_ACTIVE_POLICY_INPUT_DIM=""
    export K1_POLICY_OBS_MODE=""
    export K1_REQUESTED_POLICY_CHECKPOINT=""
    echo "[entrypoint] Policy disabled for control mode ${K1_CONTROL_MODE:-cmd_vel}"
}

select_policy_checkpoint() {
    local requested="${K1_POLICY_CHECKPOINT:-}"
    local compat="${K1_COMPAT_POLICY_CHECKPOINT:-}"

    if [[ -z "${requested}" ]]; then
        export K1_POLICY_FALLBACK_USED="0"
        export K1_ACTIVE_POLICY_CHECKPOINT=""
        export K1_ACTIVE_POLICY_INPUT_DIM=""
        export K1_POLICY_OBS_MODE="${K1_POLICY_OBS_MODE:-full_body}"
        return 0
    fi

    local selection_json
    selection_json="$(
        REQUESTED_POLICY_CHECKPOINT="${requested}" \
        COMPAT_POLICY_CHECKPOINT="${compat}" \
        /isaac-sim/python.sh - <<'PY'
import json
import os
from pathlib import Path

import torch


def checkpoint_input_dim(path_str: str) -> int | None:
    if not path_str:
        return None
    path = Path(path_str)
    if not path.is_file():
        return None
    checkpoint = torch.load(path, map_location="cpu")
    state_dict = checkpoint.get("model_state_dict", checkpoint)
    weight = state_dict.get("actor.0.weight")
    if weight is None:
        raise RuntimeError(f"{path} does not contain actor.0.weight")
    return int(weight.shape[1])


requested = os.environ.get("REQUESTED_POLICY_CHECKPOINT", "")
compat = os.environ.get("COMPAT_POLICY_CHECKPOINT", "")

supported_dims = {
    235: "locomotion_only",
    255: "full_body",
}

requested_dim = checkpoint_input_dim(requested)
compat_dim = checkpoint_input_dim(compat)

selected = ""
selected_dim = None
selected_obs_mode = ""
fallback_used = False
reason = ""

if requested_dim in supported_dims:
    selected = requested
    selected_dim = requested_dim
    selected_obs_mode = supported_dims[requested_dim]
    reason = f"requested checkpoint uses supported input dim {requested_dim}"
elif compat_dim in supported_dims:
    selected = compat
    selected_dim = compat_dim
    selected_obs_mode = supported_dims[compat_dim]
    fallback_used = True
    if requested_dim is None:
        reason = "requested checkpoint not found"
    else:
        reason = f"requested checkpoint input dim {requested_dim!r} is unsupported"
else:
    raise RuntimeError(
        "No supported K1 policy checkpoint found. "
        f"requested={requested} input_dim={requested_dim!r}, "
        f"compat={compat} input_dim={compat_dim!r}, "
        f"supported_dims={sorted(supported_dims)}"
    )

print(
    json.dumps(
        {
            "selected": selected,
            "selectedDim": selected_dim,
            "selectedObsMode": selected_obs_mode,
            "requested": requested,
            "compat": compat,
            "requestedDim": requested_dim,
            "compatDim": compat_dim,
            "fallbackUsed": fallback_used,
            "reason": reason,
        }
    )
)
PY
    )"

    export K1_ACTIVE_POLICY_CHECKPOINT="$(
        python3 -c 'import json, sys; print(json.load(sys.stdin)["selected"])' <<<"${selection_json}"
    )"
    export K1_ACTIVE_POLICY_INPUT_DIM="$(
        python3 -c 'import json, sys; print(json.load(sys.stdin)["selectedDim"])' <<<"${selection_json}"
    )"
    export K1_POLICY_FALLBACK_USED="$(
        python3 -c 'import json, sys; print("1" if json.load(sys.stdin)["fallbackUsed"] else "0")' <<<"${selection_json}"
    )"
    export K1_POLICY_OBS_MODE="$(
        python3 -c 'import json, sys; print(json.load(sys.stdin)["selectedObsMode"])' <<<"${selection_json}"
    )"
    export K1_REQUESTED_POLICY_CHECKPOINT="${requested}"

    python3 -c '
import json
import sys

payload = json.load(sys.stdin)
print(
    "[entrypoint] Policy selection: requested={} (input_dim={}), selected={} (input_dim={}, obs_mode={}), compat={} (input_dim={})".format(
        payload["requested"],
        payload["requestedDim"],
        payload["selected"],
        payload["selectedDim"],
        payload["selectedObsMode"],
        payload["compat"],
        payload["compatDim"],
    )
)
if payload["fallbackUsed"]:
    print("[entrypoint] Falling back to compat checkpoint because {}".format(payload["reason"]))
' <<<"${selection_json}"
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
    local max_tries="${3:-90}"
    for _ in $(seq 1 "${max_tries}"); do
        ensure_process_alive "${isaac_pid:-}" "Isaac runtime" || return 1
        ensure_process_alive "${rosclaw_pid:-}" "ROSClaw bringup" || return 1
        if ss -ltn 2>/dev/null | grep -q ":${port}\\b"; then
            echo "[entrypoint] ${label} ready on :${port}"
            return 0
        fi
        sleep 1
    done
    echo "[entrypoint] ${label} did not start on :${port}" >&2
    return 1
}

wait_for_topics() {
    local label="$1"
    shift
    local max_tries="${TOPIC_WAIT_TRIES:-120}"
    local missing=()

    for _ in $(seq 1 "${max_tries}"); do
        ensure_process_alive "${isaac_pid:-}" "Isaac runtime" || return 1
        ensure_process_alive "${rosclaw_pid:-}" "ROSClaw bringup" || return 1
        local topics
        topics="$(ros2 topic list --no-daemon 2>/dev/null || true)"
        missing=()
        for topic in "$@"; do
            if ! grep -qxF "${topic}" <<<"${topics}"; then
                missing+=("${topic}")
            fi
        done
        if [[ "${#missing[@]}" -eq 0 ]]; then
            echo "[entrypoint] ${label} topics ready: $*"
            return 0
        fi
        sleep 1
    done

    echo "[entrypoint] missing ${label} topics: ${missing[*]}" >&2
    return 1
}

write_ready_file() {
    python3 - <<'PY'
import json
import os
from pathlib import Path

ready_path = Path(os.environ["ROSCLAW_READY_FILE"])
control_mode = os.environ.get("K1_CONTROL_MODE", "cmd_vel")
policy_path = os.environ.get("K1_POLICY_CHECKPOINT", "") if control_mode == "cmd_vel" else ""
payload = {
    "namespace": "/k1",
    "controlMode": control_mode,
    "policyCheckpoint": os.environ.get("K1_ACTIVE_POLICY_CHECKPOINT", policy_path),
    "policyInputDim": int(os.environ.get("K1_ACTIVE_POLICY_INPUT_DIM", "0") or "0"),
    "policyObservationMode": os.environ.get("K1_POLICY_OBS_MODE", ""),
    "requestedPolicyCheckpoint": policy_path,
    "policyFallbackUsed": os.environ.get("K1_POLICY_FALLBACK_USED", "0") == "1",
    "cmdVelTopic": "/k1/cmd_vel",
    "odomTopic": "/k1/odom",
    "imuTopic": "/k1/imu",
    "jointStateTopic": "/k1/joint_states",
    "cameraTopic": "/k1/front_cam/rgb",
    "cameraInfoTopic": "/k1/front_cam/camera_info",
    "statusTopic": "/k1/booster/status",
    "livestreamMode": os.environ.get("LIVESTREAM", "2"),
    "livestreamPort": int(os.environ.get("ISAAC_LIVESTREAM_PORT", "49100")),
    "rosbridgeUrl": "ws://localhost:9090",
    "sceneModelsDir": os.environ.get("K1_SCENE_MODELS_DIR", "/workspace/scene_models"),
    "motionName": os.environ.get("K1_MOTION_NAME", "k1_mj2_seg1"),
    "freezeMotion": os.environ.get("K1_FREEZE_MOTION", "0") == "1",
    "motionStartFrame": int(os.environ.get("K1_MOTION_START_FRAME", "0") or "0"),
    "slideDisableGravity": os.environ.get("K1_SLIDE_DISABLE_GRAVITY", "0") == "1",
}
ready_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
print(f"[entrypoint] Wrote readiness file to {ready_path}")
PY
}

if [[ $# -gt 0 ]]; then
    source_system_ros
    exec "$@"
fi

namespace="$(normalize_namespace "${K1_NAMESPACE:-k1}")"
if [[ "${namespace}" != "k1" ]]; then
    echo "[entrypoint] This example expects K1_NAMESPACE=k1; got '${namespace}'." >&2
    exit 1
fi
export K1_NAMESPACE="${namespace}"

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

isaacsim_ros_lib_dir="/isaac-sim/exts/isaacsim.ros2.bridge/${ISAACSIM_INTERNAL_ROS_DISTRO}/lib"
isaac_ld_library_path="${LD_LIBRARY_PATH:-}"
if [[ -d "${isaacsim_ros_lib_dir}" ]]; then
    isaac_ld_library_path="${isaacsim_ros_lib_dir}:${isaac_ld_library_path}"
    isaac_ld_library_path="${isaac_ld_library_path%:}"
fi

rm -f "${ROSCLAW_READY_FILE}"

echo "[entrypoint] Verifying K1 assets and generating required demo npz files."
prepare_args=(
    "--convert-missing-npz"
    "--headless"
)
if [[ -n "${K1_MOTION_NAME:-}" ]]; then
    prepare_args+=("--motion-name" "${K1_MOTION_NAME}")
fi
/isaac-sim/python.sh /workspace/booster_train/scripts/prepare_k1_assets.py "${prepare_args[@]}"

if [[ "${K1_CONTROL_MODE:-cmd_vel}" == "cmd_vel" ]]; then
    select_policy_checkpoint
else
    disable_policy_checkpoint
fi

echo "[entrypoint] Starting Booster K1 demo runtime with control mode ${K1_CONTROL_MODE:-cmd_vel}"
k1_args=(
    "--namespace" "${K1_NAMESPACE}"
    "--env-count" "${K1_ENV_COUNT:-1}"
    "--terrain" "${K1_TERRAIN:-flat}"
    "--control-mode" "${K1_CONTROL_MODE:-cmd_vel}"
    "--motion-name" "${K1_MOTION_NAME:-k1_mj2_seg1}"
    "--scene-models-dir" "${K1_SCENE_MODELS_DIR:-/workspace/scene_models}"
    "--scene-models-distance" "${K1_SCENE_MODELS_DISTANCE:-2.5}"
    "--scene-models-spacing" "${K1_SCENE_MODELS_SPACING:-1.25}"
    "--scene-models-height" "${K1_SCENE_MODELS_HEIGHT:-0.0}"
    "--scene-models-scale" "${K1_SCENE_MODELS_SCALE:-0.01}"
    "--teleop-device" "${K1_TELEOP_DEVICE:-cmd_vel}"
)

if [[ "${K1_FREEZE_MOTION:-0}" == "1" ]]; then
    k1_args+=("--freeze-motion")
fi

if [[ "${K1_CONTROL_MODE:-cmd_vel}" == "cmd_vel" ]] && [[ -n "${K1_ACTIVE_POLICY_CHECKPOINT:-${K1_POLICY_CHECKPOINT:-}}" ]]; then
    k1_args+=("--policy-checkpoint" "${K1_ACTIVE_POLICY_CHECKPOINT:-${K1_POLICY_CHECKPOINT:-}}")
fi

if [[ "${K1_DISABLE_KEYBOARD_TELEOP:-0}" == "1" ]]; then
    k1_args+=("--disable-keyboard-teleop")
fi

if [[ "${K1_SDK_COMPAT:-0}" == "1" ]]; then
    k1_args+=("--sdk-compat")
fi

if [[ "${K1_USE_PLAIN_HEAD:-0}" == "1" ]]; then
    k1_args+=("--plain-head")
fi

env \
    ROS_DISTRO="${ISAACSIM_INTERNAL_ROS_DISTRO}" \
    SYSTEM_ROS_DISTRO="${SYSTEM_ROS_DISTRO}" \
    LD_LIBRARY_PATH="${isaac_ld_library_path}" \
    PUBLIC_IP="${PUBLIC_IP}" \
    /isaac-sim/python.sh /workspace/booster_train/scripts/k1_demo.py "${k1_args[@]}" &
isaac_pid=$!

if [[ "${LIVESTREAM}" != "0" ]]; then
    wait_for_port "${ISAAC_LIVESTREAM_PORT}" "Isaac livestream" 120
fi

source_system_ros

wait_for_topics "K1 runtime" \
    /k1/clock \
    /k1/cmd_vel \
    /k1/odom \
    /k1/imu \
    /k1/joint_states \
    /k1/front_cam/rgb \
    /k1/front_cam/camera_info

echo "[entrypoint] Launching ROSClaw bringup after Isaac startup"
ros2 launch rosclaw_bringup rosclaw.launch.py platform:=k1 rosbridge:=true perception:=false use_sim_time:=true &
rosclaw_pid=$!

wait_for_port 9090 "rosbridge" 90
wait_for_topics "ROSClaw bridge" /k1/booster/status

write_ready_file

if [[ "${LIVESTREAM}" != "0" ]]; then
    echo "[entrypoint] Booster K1 runtime ready: Isaac streaming on :${ISAAC_LIVESTREAM_PORT} and rosbridge is listening on :9090"
else
    echo "[entrypoint] Booster K1 runtime ready: Isaac running and rosbridge is listening on :9090"
fi

while true; do
    if ! kill -0 "${isaac_pid}" 2>/dev/null; then
        wait "${isaac_pid}"
        exit $?
    fi
    if ! kill -0 "${rosclaw_pid}" 2>/dev/null; then
        wait "${rosclaw_pid}"
        exit $?
    fi
    sleep 2
done
