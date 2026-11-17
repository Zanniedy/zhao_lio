#!/usr/bin/env bash
set -e
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [ "$#" -lt 1 ] || [ ! -f "$1/metadata.yaml" ]; then
  echo "Usage: bash scripts/replay_preprocessed.sh /path/to/rosbag2 [rviz:=false] [map2d:=false]" >&2
  exit 2
fi
bag="$(cd -- "$1" && pwd)"
shift
if [ ! -f "$root/install_preprocessing/local_setup.bash" ]; then
  echo "Build install_preprocessing first; see docs/preprocessing.md." >&2
  exit 2
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-184}"
export ROS_LOCALHOST_ONLY=1
run="$root/log_preprocessing/replay-$ROS_DOMAIN_ID"
mkdir -p "$run"
exec 9>"$run/replay.lock"
if ! flock -n 9; then
  echo "A preprocessing replay is already running in domain $ROS_DOMAIN_ID." >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
source "$root/install_preprocessing/local_setup.bash"
export ROS_LOG_DIR="$run"
printf '%s\n' "$$" > "$run/replay.pid"
exec ros2 launch dynablox_ros2 replay.launch.py "bag:=$bag" "$@"
