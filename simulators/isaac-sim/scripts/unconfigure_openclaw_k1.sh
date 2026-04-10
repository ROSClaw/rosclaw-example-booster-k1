#!/usr/bin/env bash
set -euo pipefail

OPENCLAW_BIN="${OPENCLAW_BIN:-$(command -v openclaw)}"
JQ_BIN="${JQ_BIN:-$(command -v jq)}"
OPENCLAW_CONFIG_FILE="${OPENCLAW_CONFIG_FILE:-${HOME}/.openclaw/openclaw.json}"
ROSCLAW_AUTONOMY_PLUGIN_PATH="${ROSCLAW_AUTONOMY_PLUGIN_PATH:-${HOME}/Developer/rosclaw-workspace/rosclaw-autonomy-plugin}"
SYSTEMD_DROPIN_DIR="${SYSTEMD_DROPIN_DIR:-${HOME}/.config/systemd/user/openclaw-gateway.service.d}"
HANDSHAKE_DROPIN_FILE="${HANDSHAKE_DROPIN_FILE:-${SYSTEMD_DROPIN_DIR}/k1-handshake.conf}"
ROS2_DROPIN_FILE="${ROS2_DROPIN_FILE:-${SYSTEMD_DROPIN_DIR}/ros2.conf}"

if [[ -z "${OPENCLAW_BIN}" || ! -x "${OPENCLAW_BIN}" ]]; then
    echo "[unconfigure_openclaw_k1] openclaw is not on PATH" >&2
    exit 1
fi

if [[ -z "${JQ_BIN}" || ! -x "${JQ_BIN}" ]]; then
    echo "[unconfigure_openclaw_k1] jq is not on PATH" >&2
    exit 1
fi

OPENCLAW_BIN_DIR="$(cd "$(dirname "${OPENCLAW_BIN}")" && pwd)"
OPENCLAW_DIST_DIR="${OPENCLAW_DIST_DIR:-$(cd "${OPENCLAW_BIN_DIR}/../lib/node_modules/openclaw/dist" && pwd)}"

unpatch_gateway_file() {
    local target="$1"
    if ! grep -q 'OPENCLAW_HANDSHAKE_TIMEOUT_MS' "${target}"; then
        return 0
    fi
    perl -0pi -e 's/const getHandshakeTimeoutMs = \(\) => \{\n\tconst override = process\.env\.OPENCLAW_HANDSHAKE_TIMEOUT_MS \|\| \(process\.env\.VITEST && process\.env\.OPENCLAW_TEST_HANDSHAKE_TIMEOUT_MS\);\n\tif \(override\) \{\n\t\tconst parsed = Number\(override\);\n\t\tif \(Number\.isFinite\(parsed\) && parsed > 0\) return parsed;\n\t\}\n\treturn DEFAULT_HANDSHAKE_TIMEOUT_MS;\n\};/const getHandshakeTimeoutMs = () => {\n\tif (process.env.VITEST && process.env.OPENCLAW_TEST_HANDSHAKE_TIMEOUT_MS) {\n\t\tconst parsed = Number(process.env.OPENCLAW_TEST_HANDSHAKE_TIMEOUT_MS);\n\t\tif (Number.isFinite(parsed) && parsed > 0) return parsed;\n\t}\n\treturn DEFAULT_HANDSHAKE_TIMEOUT_MS;\n};/' "${target}"
}

if [[ -f "${OPENCLAW_CONFIG_FILE}" ]]; then
    tmp_config="$(mktemp)"
    AUTONOMY_PLUGIN_PATH="${ROSCLAW_AUTONOMY_PLUGIN_PATH}" \
    "${JQ_BIN}" '
      .plugins = (.plugins // {})
      | .plugins.load = (.plugins.load // {})
      | if (.plugins.load.paths // null) then
            .plugins.load.paths |= map(select(. != env.AUTONOMY_PLUGIN_PATH))
        else
            .
        end
      | del(.plugins.entries.rosclaw.config)
      | del(.plugins.entries["rosclaw-autonomy"])
    ' "${OPENCLAW_CONFIG_FILE}" > "${tmp_config}"
    mv "${tmp_config}" "${OPENCLAW_CONFIG_FILE}"
fi

mapfile -t gateway_files < <(find "${OPENCLAW_DIST_DIR}" -maxdepth 1 -type f -name 'gateway-cli-*.js' | sort)
for gateway_file in "${gateway_files[@]}"; do
    unpatch_gateway_file "${gateway_file}"
done

rm -f "${HANDSHAKE_DROPIN_FILE}"
if [[ -f "${ROS2_DROPIN_FILE}" ]]; then
    perl -0pi -e 's/\nEnvironment=OPENCLAW_HANDSHAKE_TIMEOUT_MS=\d+\n/\n/g' "${ROS2_DROPIN_FILE}"
fi

if systemctl --user status openclaw-gateway.service >/dev/null 2>&1; then
    systemctl --user daemon-reload
    systemctl --user restart openclaw-gateway.service
fi

"${OPENCLAW_BIN}" config validate >/dev/null

echo "[unconfigure_openclaw_k1] removed K1-specific OpenClaw config"
echo "[unconfigure_openclaw_k1] auth profiles and channel credentials were left intact"
