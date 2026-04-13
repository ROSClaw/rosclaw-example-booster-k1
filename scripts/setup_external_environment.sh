#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

FORCE_VENDOR_LINKS=0
SYNC_OPENCLAW=1
REAL_HARDWARE_ONLY=0

usage() {
    cat <<'EOF'
Usage: ./scripts/setup_external_environment.sh [options]

Prepare external repositories needed by the Booster K1 Isaac Sim example.
Use --real-hardware-only to initialize only the real-hardware submodules and
leave the optional simulator/OpenClaw development submodules untouched.

Options:
  --real-hardware-only      Initialize only the real-hardware submodules and skip
                            simulator vendor links, submodule patching, and
                            OpenClaw extension sync.
  --force-vendor-links      Move existing local vendor copies aside and replace them with submodule symlinks.
  --skip-openclaw-sync      Do not copy the patched rosclaw-plugin submodule into the local OpenClaw extension.
  -h, --help                Show this help text.

Environment:
  OPENCLAW_ROSCLAW_EXTENSION_DIR  OpenClaw rosclaw extension install path.
                                  Defaults to ~/.openclaw/extensions/rosclaw.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --force-vendor-links)
            FORCE_VENDOR_LINKS=1
            shift
            ;;
        --real-hardware-only)
            REAL_HARDWARE_ONLY=1
            shift
            ;;
        --skip-openclaw-sync)
            SYNC_OPENCLAW=0
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "[setup_external_environment] unknown argument: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

require_file() {
    local path="$1"
    if [[ ! -e "${path}" ]]; then
        echo "[setup_external_environment] missing required path: ${path}" >&2
        exit 1
    fi
}

ensure_submodules() {
    echo "[setup_external_environment] initializing submodules"
    if [[ "${REAL_HARDWARE_ONLY}" == "1" ]]; then
        git -C "${REPO_ROOT}" submodule update --init -- \
            real-hardware/src/rosclaw-ros2-autonomy \
            real-hardware/src/booster_robotics_sdk_ros2
        return 0
    fi
    git -C "${REPO_ROOT}" submodule update --init -- \
        external/booster-assets \
        external/booster-k1-rl \
        external/rosclaw-ros2 \
        external/rosclaw-plugin \
        real-hardware/src/rosclaw-ros2-autonomy \
        real-hardware/src/booster_robotics_sdk_ros2
}

relative_symlink() {
    local target="$1"
    local link_path="$2"
    local label="$3"

    if [[ -L "${link_path}" ]]; then
        local current
        current="$(readlink "${link_path}")"
        if [[ "${current}" == "${target}" ]]; then
            echo "[setup_external_environment] ${label} link already configured"
            return 0
        fi
        if [[ "${FORCE_VENDOR_LINKS}" != "1" ]]; then
            echo "[setup_external_environment] ${label} link points at ${current}; use --force-vendor-links to replace it" >&2
            return 1
        fi
        rm "${link_path}"
    elif [[ -e "${link_path}" ]]; then
        if [[ "${FORCE_VENDOR_LINKS}" != "1" ]]; then
            echo "[setup_external_environment] ${label} exists at ${link_path}; use --force-vendor-links to move it aside" >&2
            return 1
        fi
        local backup="${link_path}.bak.$(date +%Y%m%d%H%M%S)"
        echo "[setup_external_environment] moving existing ${label} to ${backup}"
        mv "${link_path}" "${backup}"
    fi

    ln -s "${target}" "${link_path}"
    echo "[setup_external_environment] linked ${label}: ${link_path} -> ${target}"
}

configure_vendor_links() {
    local vendor_dir="${REPO_ROOT}/isaac-sim-runtime/vendor"
    mkdir -p "${vendor_dir}"

    require_file "${REPO_ROOT}/external/booster-assets"
    require_file "${REPO_ROOT}/external/booster-k1-rl/source/booster_train"

    relative_symlink "../../external/booster-assets" "${vendor_dir}/booster_assets" "Booster K1 assets"
    relative_symlink "../../external/booster-k1-rl" "${vendor_dir}/booster_train" "Booster K1 training/runtime source"
}

apply_patch_once() {
    local target_repo="$1"
    local patch_file="$2"
    local label="$3"

    require_file "${target_repo}/.git"
    require_file "${patch_file}"

    if git -C "${target_repo}" apply --reverse --check "${patch_file}" >/dev/null 2>&1; then
        echo "[setup_external_environment] ${label} patch already applied"
        return 0
    fi

    if [[ -n "$(git -C "${target_repo}" status --porcelain)" ]]; then
        echo "[setup_external_environment] ${label} repo has local changes; refusing to patch automatically: ${target_repo}" >&2
        echo "[setup_external_environment] clean that repo or apply ${patch_file} manually" >&2
        exit 1
    fi

    git -C "${target_repo}" apply --check "${patch_file}"
    git -C "${target_repo}" apply "${patch_file}"
    echo "[setup_external_environment] applied ${label} patch"
}

sync_openclaw_extension() {
    if [[ "${SYNC_OPENCLAW}" != "1" ]]; then
        return 0
    fi

    local source_dir="${REPO_ROOT}/external/rosclaw-plugin"
    local target_dir="${OPENCLAW_ROSCLAW_EXTENSION_DIR:-${HOME}/.openclaw/extensions/rosclaw}"

    if [[ ! -d "${target_dir}" ]]; then
        echo "[setup_external_environment] OpenClaw rosclaw extension not found at ${target_dir}; skipping sync"
        return 0
    fi

    if command -v realpath >/dev/null 2>&1 && [[ "$(realpath "${source_dir}")" == "$(realpath "${target_dir}")" ]]; then
        echo "[setup_external_environment] OpenClaw extension dir is the submodule; sync not needed"
        return 0
    fi

    if ! command -v rsync >/dev/null 2>&1; then
        echo "[setup_external_environment] rsync is required to sync OpenClaw extension files" >&2
        exit 1
    fi

    rsync -a \
        --exclude .git \
        --exclude node_modules \
        --exclude .turbo \
        "${source_dir}/" \
        "${target_dir}/"
    echo "[setup_external_environment] synced patched rosclaw-plugin to ${target_dir}"
}

ensure_submodules
if [[ "${REAL_HARDWARE_ONLY}" == "1" ]]; then
    cat <<'EOF'
[setup_external_environment] real-hardware submodules prepared

Initialized:
  real-hardware/src/booster_robotics_sdk_ros2
  real-hardware/src/rosclaw-ros2-autonomy

No simulator/OpenClaw external submodules were patched or synced.
EOF
    exit 0
fi

configure_vendor_links
apply_patch_once \
    "${REPO_ROOT}/external/booster-k1-rl" \
    "${REPO_ROOT}/patches/booster-k1-rl-runtime-overrides.patch" \
    "Booster K1 RL runtime overrides"
apply_patch_once \
    "${REPO_ROOT}/external/rosclaw-ros2" \
    "${REPO_ROOT}/patches/rosclaw-ros2-k1-bringup.patch" \
    "rosclaw-ros2 K1 bringup"
apply_patch_once \
    "${REPO_ROOT}/external/rosclaw-plugin" \
    "${REPO_ROOT}/patches/rosclaw-plugin-k1-openclaw.patch" \
    "rosclaw-plugin K1 OpenClaw"
sync_openclaw_extension

cat <<'EOF'
[setup_external_environment] external environment prepared

Next:
  ./simulators/isaac-sim/scripts/configure_openclaw_k1.sh
  ./simulators/isaac-sim/scripts/run_k1_isaac_sim.sh --mode webrtc

The setup patches intentionally modify submodule working trees. Do not commit
those submodule working-tree modifications; keep the patch files in this repo
as the reproducible source of those changes.
EOF
