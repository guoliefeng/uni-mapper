#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 4 ]]; then
  echo "Usage: $0 <catkin-install-setup.bash> <config-dir> <run-root> <resource-file>" >&2
  exit 2
fi

setup_file=$1
config_dir=$2
run_root=$3
resource_file=$4

if [[ ! -f "$setup_file" || ! -f "$config_dir/config.json" ]]; then
  echo "Invalid setup file or config directory" >&2
  exit 2
fi

mkdir -p "$run_root" "$(dirname "$resource_file")"
# shellcheck disable=SC1090
source "$setup_file"
export ROS_LOG_DIR="${TMPDIR:-/tmp}/uni_mapper_dynamic_removal_roslogs"
mkdir -p "$ROS_LOG_DIR"

/usr/bin/time -v -o "$resource_file" \
  roslaunch open_lmm_ros open_lmm.launch config_path:="$config_dir"

latest_run=$(find "$run_root" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' \
  | sort -nr | head -n 1 | cut -d' ' -f2-)
if [[ -z "$latest_run" || ! -f "$latest_run/global_map_A.pcd" ]]; then
  echo "MapServer did not produce global_map_A.pcd under $run_root" >&2
  exit 1
fi
printf '%s\n' "$latest_run"
