#!/usr/bin/env bash
set -euo pipefail

containers=(
    rosclaw-example-booster-k1
    rosclaw-example-booster-k1-webrtc
    rosclaw-example-booster-k1-headless
    rosclaw-example-booster-k1-gui
)

for container in "${containers[@]}"; do
    if docker ps -a --format '{{.Names}}' | grep -qx "${container}"; then
        docker stop "${container}" >/dev/null 2>&1 || true
    fi
done

echo "[stop_k1_isaac_sim] requested stop for known K1 Isaac Sim containers"
