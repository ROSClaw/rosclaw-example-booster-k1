#!/usr/bin/env bash
set -euo pipefail

HOST="${HOST:-127.0.0.1}"
PORT="${PORT:-9090}"

python3 - <<'PY'
import os
import socket
import sys

host = os.environ.get("HOST", "127.0.0.1")
port = int(os.environ.get("PORT", "9090"))

with socket.create_connection((host, port), timeout=5):
    print(f"rosbridge reachable at ws://{host}:{port}")

sys.exit(0)
PY
