#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"

READY_FILE="${READY_FILE:-/tmp/rosclaw-k1-ready.json}"

container_bash "
set -euo pipefail
if [[ ! -f \"${READY_FILE}\" ]]; then
    echo \"missing readiness file: ${READY_FILE}\" >&2
    exit 1
fi
python3 - <<'PY'
import json
from pathlib import Path

ready_path = Path('${READY_FILE}')
payload = json.loads(ready_path.read_text(encoding='utf-8'))
print(f\"Readiness file: {ready_path}\")
print(f\"namespace: {payload['namespace']}\")
print(f\"control mode: {payload['controlMode']}\")
print(f\"policy checkpoint: {payload['policyCheckpoint']}\")
print(f\"requested policy checkpoint: {payload['requestedPolicyCheckpoint']}\")
print(f\"policy fallback used: {payload['policyFallbackUsed']}\")
print(f\"freeze motion: {payload.get('freezeMotion')}\")
print(f\"motion start frame: {payload.get('motionStartFrame')}\")
print(f\"slide disable gravity: {payload.get('slideDisableGravity')}\")
print(f\"cmd_vel topic: {payload['cmdVelTopic']}\")
print(f\"odom topic: {payload['odomTopic']}\")
print(f\"camera topic: {payload['cameraTopic']}\")
print(f\"status topic: {payload['statusTopic']}\")
print(f\"livestream mode: {payload['livestreamMode']}\")
print(f\"livestream port: {payload['livestreamPort']}\")
if payload['controlMode'] == 'cmd_vel':
    if not payload['requestedPolicyCheckpoint'].endswith('walking_policy_latest.pt'):
        raise SystemExit('unexpected requested policy checkpoint in readiness file')
    if not (
        payload['policyCheckpoint'].endswith('walking_policy_latest.pt')
        or payload['policyCheckpoint'].endswith('k1_cmd_vel.pt')
    ):
        raise SystemExit('unexpected active policy checkpoint in readiness file')
elif payload['controlMode'] == 'motion_replay':
    if payload['policyCheckpoint'] or payload['requestedPolicyCheckpoint']:
        raise SystemExit('policy should be disabled in motion_replay mode')
    if payload.get('freezeMotion') is not True:
        raise SystemExit('motion_replay example should run with freezeMotion enabled')
    if payload.get('slideDisableGravity') is not True:
        raise SystemExit('motion_replay example should disable gravity on the K1')
else:
    raise SystemExit(f\"unexpected control mode: {payload['controlMode']}\")
PY
"
