#!/usr/bin/env bash

set -euo pipefail

ROBOT_HOST="${K1_ROBOT_HOST:-booster@192.168.2.152}"
SSH_PASSWORD="${K1_SSH_PASSWORD:-}"
SUDO_PASSWORD="${K1_SUDO_PASSWORD:-${SSH_PASSWORD}}"
VERIFY_SECONDS="${K1_VERIFY_SECONDS:-45}"
CHECK_ONLY="false"
RESTART_SERVICES="true"
DRY_RUN="false"
SSH_OPTS=(-o StrictHostKeyChecking=no)

PATCH_FILES=(
  /opt/booster/BoosterRos2/start_rpc_service.sh
  /opt/booster/BoosterShell/start_joystick.sh
  /opt/booster/BoosterAgent/start.sh
  /opt/booster/BoosterLui/bin/start_client.sh
  /opt/booster/RTCCli/bin/start_client.sh
  /opt/booster/DaemonPerception/bin/start.sh
)

SYSTEMD_SERVICES=(
  booster-daemon.service
  booster-agent-manager.service
  booster-lui.service
  joystick_ros2.service
  booster-daemon-perception.service
  booster-rtc-speech.service
)

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Patch the live K1 to use Booster's UDP-only Fast DDS profile and optionally
restart the affected services.

Options:
  --robot HOST           SSH target for the K1.
                         Default: ${ROBOT_HOST}
  --ssh-password PASS    Non-interactive SSH password for the K1.
                         Requires sshpass on the local host.
  --sudo-password PASS   Non-interactive sudo password for the K1.
                         Defaults to the SSH password when not set.
  --check-only           Inspect current profile wiring and service state
                         without changing the robot.
  --no-restart           Patch files only; do not restart services.
  --verify-seconds N     Stability hold after restart before collecting
                         service and journal state.
                         Default: ${VERIFY_SECONDS}
  --dry-run              Print the ssh command without executing it.
  -h, --help             Show this help text.

Environment:
  K1_ROBOT_HOST          Default SSH target for the K1.
  K1_SSH_PASSWORD        Optional non-interactive SSH password.
  K1_SUDO_PASSWORD       Optional non-interactive sudo password.
  K1_VERIFY_SECONDS      Default verification hold after restart.
EOF
}

log() {
  printf '[k1-fastdds-fix] %s\n' "$*"
}

die() {
  printf '[k1-fastdds-fix] ERROR: %s\n' "$*" >&2
  exit 1
}

run_remote_root_script() {
  local remote_script="$1"
  local ssh_cmd=(ssh "${SSH_OPTS[@]}")

  if [[ "${DRY_RUN}" == "true" ]]; then
    if [[ -n "${SSH_PASSWORD}" ]]; then
      ssh_cmd=(sshpass -p "${SSH_PASSWORD}" ssh -o PubkeyAuthentication=no -o PreferredAuthentications=password "${SSH_OPTS[@]}")
    fi
    printf '[dry-run]'
    printf ' %q' "${ssh_cmd[@]}"
    printf ' %q' "${ROBOT_HOST}"
    if [[ -n "${SUDO_PASSWORD}" ]]; then
      printf ' %q' "sudo -S bash -s --"
    else
      printf ' %q' "sudo bash -s --"
    fi
    printf '\n'
    return 0
  fi

  if [[ -n "${SSH_PASSWORD}" ]]; then
    command -v sshpass >/dev/null 2>&1 || die "sshpass is required when --ssh-password or K1_SSH_PASSWORD is set"
    ssh_cmd=(sshpass -p "${SSH_PASSWORD}" ssh -o PubkeyAuthentication=no -o PreferredAuthentications=password "${SSH_OPTS[@]}")
  fi

  if [[ -n "${SUDO_PASSWORD}" ]]; then
    {
      printf '%s\n' "${SUDO_PASSWORD}"
      printf '%s\n' "${remote_script}"
    } | "${ssh_cmd[@]}" "${ROBOT_HOST}" "sudo -S bash -s --"
    return 0
  fi

  "${ssh_cmd[@]}" -tt "${ROBOT_HOST}" "sudo bash -s --" <<<"${remote_script}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot)
      [[ $# -ge 2 ]] || die "--robot requires a value"
      ROBOT_HOST="$2"
      shift 2
      ;;
    --ssh-password)
      [[ $# -ge 2 ]] || die "--ssh-password requires a value"
      SSH_PASSWORD="$2"
      if [[ -z "${SUDO_PASSWORD}" ]]; then
        SUDO_PASSWORD="${SSH_PASSWORD}"
      fi
      shift 2
      ;;
    --sudo-password)
      [[ $# -ge 2 ]] || die "--sudo-password requires a value"
      SUDO_PASSWORD="$2"
      shift 2
      ;;
    --check-only)
      CHECK_ONLY="true"
      shift
      ;;
    --no-restart)
      RESTART_SERVICES="false"
      shift
      ;;
    --verify-seconds)
      [[ $# -ge 2 ]] || die "--verify-seconds requires a value"
      VERIFY_SECONDS="$2"
      shift 2
      ;;
    --dry-run)
      DRY_RUN="true"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown argument: $1"
      ;;
  esac
done

if ! [[ "${VERIFY_SECONDS}" =~ ^[0-9]+$ ]]; then
  die "--verify-seconds must be a non-negative integer"
fi

read -r -d '' REMOTE_SCRIPT <<EOF || true
set -euo pipefail

PROFILE_FROM="/opt/booster/BoosterRos2/fastdds_profile.xml"
PROFILE_TO="/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml"
CHECK_ONLY="${CHECK_ONLY}"
RESTART_SERVICES="${RESTART_SERVICES}"
VERIFY_SECONDS="${VERIFY_SECONDS}"

PATCH_FILES=(
$(printf '  %q\n' "${PATCH_FILES[@]}")
)

SYSTEMD_SERVICES=(
$(printf '  %q\n' "${SYSTEMD_SERVICES[@]}")
)

for path in "\${PATCH_FILES[@]}"; do
  [[ -f "\${path}" ]] || { echo "[remote] missing required file: \${path}" >&2; exit 1; }
done

echo "[remote] current Fast DDS profile wiring"
grep -Hn 'FASTRTPS_DEFAULT_PROFILES_FILE' "\${PATCH_FILES[@]}" || true

if [[ "\${CHECK_ONLY}" == "true" ]]; then
  echo "[remote] service state"
  SYSTEMD_PAGER=cat systemctl show "\${SYSTEMD_SERVICES[@]}" -p Id -p ActiveState -p SubState -p NRestarts -p MemoryCurrent
  echo "[remote] memory"
  free -h
  exit 0
fi

stamp="\$(date +%Y%m%d-%H%M%S)"
for path in "\${PATCH_FILES[@]}"; do
  cp "\${path}" "\${path}.bak-\${stamp}"
  sed -i "s#\${PROFILE_FROM}#\${PROFILE_TO}#g" "\${path}"
done

echo "[remote] patched files with backup stamp \${stamp}"
grep -Hn 'FASTRTPS_DEFAULT_PROFILES_FILE' "\${PATCH_FILES[@]}"

verify_start=""
if [[ "\${RESTART_SERVICES}" == "true" ]]; then
  systemctl restart "\${SYSTEMD_SERVICES[@]}"
  verify_start="\$(date '+%Y-%m-%d %H:%M:%S')"
  echo "[remote] restarted services at \${verify_start}"
  if [[ "\${VERIFY_SECONDS}" -gt 0 ]]; then
    sleep "\${VERIFY_SECONDS}"
  fi
fi

echo "[remote] service state"
SYSTEMD_PAGER=cat systemctl show "\${SYSTEMD_SERVICES[@]}" -p Id -p ActiveState -p SubState -p NRestarts -p MemoryCurrent
echo "[remote] memory"
free -h

if [[ -n "\${verify_start}" ]]; then
  echo "[remote] error scan since \${verify_start}"
  journalctl -b --since "\${verify_start}" --no-pager | egrep 'Out of memory|oom-killer|Failed with result .oom-kill.|RTPS_TRANSPORT_SHM|ParticipantEntitiesInfo|booster-daemon.service: A process of this unit has been killed by the OOM killer' || true
fi
EOF

log "Applying K1 Fast DDS UDP-only profile fix on ${ROBOT_HOST}"
run_remote_root_script "${REMOTE_SCRIPT}"
