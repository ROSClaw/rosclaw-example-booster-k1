#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"

LIVESTREAM_PORT="${LIVESTREAM_PORT:-${ISAAC_LIVESTREAM_PORT:-49100}}"

container_bash "
set -euo pipefail
if ! ss -ltn 2>/dev/null | grep -q ':${LIVESTREAM_PORT}\b'; then
    echo 'missing Isaac livestream listener on :${LIVESTREAM_PORT}' >&2
    exit 1
fi
echo 'Isaac livestream listener reachable on :${LIVESTREAM_PORT}'
"
