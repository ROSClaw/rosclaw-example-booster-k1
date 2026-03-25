#!/usr/bin/env bash

# Source this file:
#   source ./setup-dds-env.sh
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "Source this file instead of executing it:" >&2
  echo "  source ${BASH_SOURCE[0]}" >&2
  exit 1
fi

_rosclaw_example_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_real_hw_ws="${_rosclaw_example_root}/real-hardware"

_clean_path=""
IFS=':' read -r -a _path_entries <<< "${PATH:-}"
for _entry in "${_path_entries[@]}"; do
  case "${_entry}" in
    *anaconda3*|*miniconda*|*condabin*)
      continue
      ;;
  esac
  if [[ -z "${_entry}" ]]; then
    continue
  fi
  if [[ -z "${_clean_path}" ]]; then
    _clean_path="${_entry}"
  else
    _clean_path="${_clean_path}:${_entry}"
  fi
done

unset CONDA_EXE CONDA_PREFIX CONDA_PROMPT_MODIFIER CONDA_SHLVL
unset CONDA_PYTHON_EXE CONDA_DEFAULT_ENV VIRTUAL_ENV PYTHONHOME PYTHONPATH
unset CYCLONEDDS_URI

export PATH="${_clean_path:-/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin}"

source /opt/ros/humble/setup.bash
if [[ -f "${_real_hw_ws}/install/setup.bash" ]]; then
  source "${_real_hw_ws}/install/setup.bash"
fi

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE="${_rosclaw_example_root}/dds-profile.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="${_rosclaw_example_root}/dds-profile.xml"

echo "Configured ROS 2 Humble + Fast DDS environment"
echo "  project root: ${_rosclaw_example_root}"
echo "  workspace:    ${_real_hw_ws}"
echo "  RMW:          ${RMW_IMPLEMENTATION}"
echo "  profile:      ${FASTRTPS_DEFAULT_PROFILES_FILE}"
echo "  python:       $(command -v python3)"
