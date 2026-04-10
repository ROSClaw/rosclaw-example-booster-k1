#!/usr/bin/env bash
set -euo pipefail

OPENCLAW_BIN="${OPENCLAW_BIN:-$(command -v openclaw)}"
JQ_BIN="${JQ_BIN:-$(command -v jq)}"
OPENCLAW_CONFIG_FILE="${OPENCLAW_CONFIG_FILE:-${HOME}/.openclaw/openclaw.json}"
ROSCLAW_AUTONOMY_PLUGIN_PATH="${ROSCLAW_AUTONOMY_PLUGIN_PATH:-${HOME}/Developer/rosclaw-workspace/rosclaw-autonomy-plugin}"
ENABLE_ROSCLAW_AUTONOMY_PLUGIN="${ENABLE_ROSCLAW_AUTONOMY_PLUGIN:-false}"
ROSBRIDGE_URL="${ROSBRIDGE_URL:-ws://127.0.0.1:9090}"
ROBOT_NAME="${ROBOT_NAME:-Booster K1}"
ROBOT_NAMESPACE="${ROBOT_NAMESPACE:-/k1}"
OPENCLAW_HANDSHAKE_TIMEOUT_MS="${OPENCLAW_HANDSHAKE_TIMEOUT_MS:-10000}"
SYSTEMD_DROPIN_DIR="${SYSTEMD_DROPIN_DIR:-${HOME}/.config/systemd/user/openclaw-gateway.service.d}"
HANDSHAKE_DROPIN_FILE="${HANDSHAKE_DROPIN_FILE:-${SYSTEMD_DROPIN_DIR}/k1-handshake.conf}"
ROS2_DROPIN_FILE="${ROS2_DROPIN_FILE:-${SYSTEMD_DROPIN_DIR}/ros2.conf}"
MAX_LINEAR_VELOCITY="${MAX_LINEAR_VELOCITY:-1}"
MAX_ANGULAR_VELOCITY="${MAX_ANGULAR_VELOCITY:-1.5}"
WORKSPACE_X_MIN="${WORKSPACE_X_MIN:--10}"
WORKSPACE_X_MAX="${WORKSPACE_X_MAX:-10}"
WORKSPACE_Y_MIN="${WORKSPACE_Y_MIN:--10}"
WORKSPACE_Y_MAX="${WORKSPACE_Y_MAX:-10}"
AUTONOMY_STARTUP_MODE="${AUTONOMY_STARTUP_MODE:-FULL_AUTONOMY}"
AUTONOMY_TICK_HZ="${AUTONOMY_TICK_HZ:-1}"
AUTONOMY_IDLE_ROAM_STRATEGY="${AUTONOMY_IDLE_ROAM_STRATEGY:-frontier}"
AUTONOMY_DEFAULT_EXECUTOR="${AUTONOMY_DEFAULT_EXECUTOR:-auto}"
AUTONOMY_MANUAL_RETURN_DELAY_SEC="${AUTONOMY_MANUAL_RETURN_DELAY_SEC:-5}"
AUTONOMY_LOW_BATTERY_PCT="${AUTONOMY_LOW_BATTERY_PCT:-20}"
AUTONOMY_STUCK_TIMEOUT_SEC="${AUTONOMY_STUCK_TIMEOUT_SEC:-15}"
AUTONOMY_MAX_TOOL_CALLS_PER_MINUTE="${AUTONOMY_MAX_TOOL_CALLS_PER_MINUTE:-30}"

if [[ -z "${OPENCLAW_BIN}" || ! -x "${OPENCLAW_BIN}" ]]; then
    echo "[configure_openclaw_k1] openclaw is not on PATH" >&2
    exit 1
fi

if [[ -z "${JQ_BIN}" || ! -x "${JQ_BIN}" ]]; then
    echo "[configure_openclaw_k1] jq is not on PATH" >&2
    exit 1
fi

OPENCLAW_BIN_DIR="$(cd "$(dirname "${OPENCLAW_BIN}")" && pwd)"
OPENCLAW_DIST_DIR="${OPENCLAW_DIST_DIR:-$(cd "${OPENCLAW_BIN_DIR}/../lib/node_modules/openclaw/dist" && pwd)}"

backup_file() {
    local target="$1"
    if [[ -f "${target}" ]]; then
        cp "${target}" "${target}.bak.$(date +%Y%m%d%H%M%S)"
    fi
}

patch_gateway_file() {
    local target="$1"
    if grep -q 'OPENCLAW_HANDSHAKE_TIMEOUT_MS' "${target}"; then
        return 0
    fi
    perl -0pi -e 's/const getHandshakeTimeoutMs = \(\) => \{\n\tif \(process\.env\.VITEST && process\.env\.OPENCLAW_TEST_HANDSHAKE_TIMEOUT_MS\) \{\n\t\tconst parsed = Number\(process\.env\.OPENCLAW_TEST_HANDSHAKE_TIMEOUT_MS\);\n\t\tif \(Number\.isFinite\(parsed\) && parsed > 0\) return parsed;\n\t\}\n\treturn DEFAULT_HANDSHAKE_TIMEOUT_MS;\n\};/const getHandshakeTimeoutMs = () => {\n\tconst override = process.env.OPENCLAW_HANDSHAKE_TIMEOUT_MS || (process.env.VITEST && process.env.OPENCLAW_TEST_HANDSHAKE_TIMEOUT_MS);\n\tif (override) {\n\t\tconst parsed = Number(override);\n\t\tif (Number.isFinite(parsed) && parsed > 0) return parsed;\n\t}\n\treturn DEFAULT_HANDSHAKE_TIMEOUT_MS;\n};/' "${target}"
    if ! grep -q 'OPENCLAW_HANDSHAKE_TIMEOUT_MS' "${target}"; then
        echo "[configure_openclaw_k1] failed to patch ${target}" >&2
        exit 1
    fi
}

mkdir -p "$(dirname "${OPENCLAW_CONFIG_FILE}")" "${SYSTEMD_DROPIN_DIR}"
if [[ ! -f "${OPENCLAW_CONFIG_FILE}" ]]; then
    printf '{}\n' > "${OPENCLAW_CONFIG_FILE}"
fi

backup_file "${OPENCLAW_CONFIG_FILE}"

tmp_config="$(mktemp)"
AUTONOMY_PLUGIN_PATH="${ROSCLAW_AUTONOMY_PLUGIN_PATH}" \
ENABLE_ROSCLAW_AUTONOMY_PLUGIN="${ENABLE_ROSCLAW_AUTONOMY_PLUGIN}" \
ROSBRIDGE_URL="${ROSBRIDGE_URL}" \
ROBOT_NAME="${ROBOT_NAME}" \
ROBOT_NAMESPACE="${ROBOT_NAMESPACE}" \
MAX_LINEAR_VELOCITY="${MAX_LINEAR_VELOCITY}" \
MAX_ANGULAR_VELOCITY="${MAX_ANGULAR_VELOCITY}" \
WORKSPACE_X_MIN="${WORKSPACE_X_MIN}" \
WORKSPACE_X_MAX="${WORKSPACE_X_MAX}" \
WORKSPACE_Y_MIN="${WORKSPACE_Y_MIN}" \
WORKSPACE_Y_MAX="${WORKSPACE_Y_MAX}" \
AUTONOMY_STARTUP_MODE="${AUTONOMY_STARTUP_MODE}" \
AUTONOMY_TICK_HZ="${AUTONOMY_TICK_HZ}" \
AUTONOMY_IDLE_ROAM_STRATEGY="${AUTONOMY_IDLE_ROAM_STRATEGY}" \
AUTONOMY_DEFAULT_EXECUTOR="${AUTONOMY_DEFAULT_EXECUTOR}" \
AUTONOMY_MANUAL_RETURN_DELAY_SEC="${AUTONOMY_MANUAL_RETURN_DELAY_SEC}" \
AUTONOMY_LOW_BATTERY_PCT="${AUTONOMY_LOW_BATTERY_PCT}" \
AUTONOMY_STUCK_TIMEOUT_SEC="${AUTONOMY_STUCK_TIMEOUT_SEC}" \
AUTONOMY_MAX_TOOL_CALLS_PER_MINUTE="${AUTONOMY_MAX_TOOL_CALLS_PER_MINUTE}" \
"${JQ_BIN}" '
  .plugins = (.plugins // {})
  | .plugins.load = (.plugins.load // {})
  | .plugins.load.paths =
      if (env.ENABLE_ROSCLAW_AUTONOMY_PLUGIN == "true") then
        (((.plugins.load.paths // []) + [env.AUTONOMY_PLUGIN_PATH]) | unique)
      else
        ((.plugins.load.paths // []) | map(select(. != env.AUTONOMY_PLUGIN_PATH)))
      end
  | .plugins.entries = (.plugins.entries // {})
  | .plugins.entries.rosclaw = ((.plugins.entries.rosclaw // {}) + {
      enabled: true,
      config: {
        transport: {mode: "rosbridge"},
        rosbridge: {
          url: env.ROSBRIDGE_URL,
          reconnect: true,
          reconnectInterval: 3000
        },
        robot: {
          name: env.ROBOT_NAME,
          namespace: env.ROBOT_NAMESPACE
        },
        safety: {
          maxLinearVelocity: (env.MAX_LINEAR_VELOCITY | tonumber),
          maxAngularVelocity: (env.MAX_ANGULAR_VELOCITY | tonumber),
          workspaceLimits: {
            xMin: (env.WORKSPACE_X_MIN | tonumber),
            xMax: (env.WORKSPACE_X_MAX | tonumber),
            yMin: (env.WORKSPACE_Y_MIN | tonumber),
            yMax: (env.WORKSPACE_Y_MAX | tonumber)
          }
        }
      }
    })
  | if (env.ENABLE_ROSCLAW_AUTONOMY_PLUGIN == "true") then
      .plugins.entries["rosclaw-autonomy"] = ((.plugins.entries["rosclaw-autonomy"] // {}) + {
        enabled: true,
        config: {
          transport: {mode: "rosbridge"},
          robot: {
            name: env.ROBOT_NAME,
            namespace: env.ROBOT_NAMESPACE
          },
          autonomy: {
            enabled: true,
            startupMode: env.AUTONOMY_STARTUP_MODE,
            tickHz: (env.AUTONOMY_TICK_HZ | tonumber),
            idleRoamStrategy: env.AUTONOMY_IDLE_ROAM_STRATEGY,
            defaultExecutor: env.AUTONOMY_DEFAULT_EXECUTOR,
            manualReturnDelaySec: (env.AUTONOMY_MANUAL_RETURN_DELAY_SEC | tonumber),
            lowBatteryPct: (env.AUTONOMY_LOW_BATTERY_PCT | tonumber),
            stuckTimeoutSec: (env.AUTONOMY_STUCK_TIMEOUT_SEC | tonumber),
            maxToolCallsPerMinute: (env.AUTONOMY_MAX_TOOL_CALLS_PER_MINUTE | tonumber)
          },
          rosbridge: {
            url: env.ROSBRIDGE_URL,
            reconnect: true,
            reconnectInterval: 3000
          },
          safety: {
            maxLinearVelocity: (env.MAX_LINEAR_VELOCITY | tonumber),
            maxAngularVelocity: (env.MAX_ANGULAR_VELOCITY | tonumber),
            allowlistedTopics: [],
            allowlistedServices: [],
            allowlistedActions: [],
            workspaceLimits: {
              xMin: (env.WORKSPACE_X_MIN | tonumber),
              xMax: (env.WORKSPACE_X_MAX | tonumber),
              yMin: (env.WORKSPACE_Y_MIN | tonumber),
              yMax: (env.WORKSPACE_Y_MAX | tonumber)
            }
          }
        }
      })
    else
      del(.plugins.entries["rosclaw-autonomy"])
    end
' "${OPENCLAW_CONFIG_FILE}" > "${tmp_config}"
mv "${tmp_config}" "${OPENCLAW_CONFIG_FILE}"

mapfile -t gateway_files < <(find "${OPENCLAW_DIST_DIR}" -maxdepth 1 -type f -name 'gateway-cli-*.js' | sort)
if [[ "${#gateway_files[@]}" -eq 0 ]]; then
    echo "[configure_openclaw_k1] no gateway-cli files found in ${OPENCLAW_DIST_DIR}" >&2
    exit 1
fi

for gateway_file in "${gateway_files[@]}"; do
    backup_file "${gateway_file}"
    patch_gateway_file "${gateway_file}"
done

cat > "${HANDSHAKE_DROPIN_FILE}" <<EOF
[Service]
Environment=OPENCLAW_HANDSHAKE_TIMEOUT_MS=${OPENCLAW_HANDSHAKE_TIMEOUT_MS}
EOF

if [[ -f "${ROS2_DROPIN_FILE}" ]]; then
    perl -0pi -e 's/\nEnvironment=OPENCLAW_HANDSHAKE_TIMEOUT_MS=\d+\n/\n/g' "${ROS2_DROPIN_FILE}"
fi

if systemctl --user status openclaw-gateway.service >/dev/null 2>&1; then
    systemctl --user daemon-reload
    systemctl --user restart openclaw-gateway.service
fi

"${OPENCLAW_BIN}" config validate >/dev/null
"${OPENCLAW_BIN}" gateway health >/dev/null

echo "[configure_openclaw_k1] configured OpenClaw for Booster K1"
echo "[configure_openclaw_k1] config file: ${OPENCLAW_CONFIG_FILE}"
echo "[configure_openclaw_k1] handshake drop-in: ${HANDSHAKE_DROPIN_FILE}"
if [[ "${ENABLE_ROSCLAW_AUTONOMY_PLUGIN}" == "true" ]]; then
    echo "[configure_openclaw_k1] rosclaw-autonomy enabled"
else
    echo "[configure_openclaw_k1] rosclaw-autonomy left disabled for the Isaac Sim K1 profile"
fi
