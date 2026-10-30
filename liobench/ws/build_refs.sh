#!/usr/bin/env bash
# 编译对比用的参考算法（Point-LIO、Batch-LIO、small_point_lio、Super-LIO、genz-lio）。
# 源码不在本仓库里：默认从本仓库的上一级目录找（liop/），可以用 LIOBENCH_REFS 指定。
#   用法：liobench/ws/build_refs.sh
set -euo pipefail
WS="$(cd "$(dirname "$0")" && pwd)"
REFS="${LIOBENCH_REFS:-$(cd "$WS/../../.." && pwd)}"
LIVOX="${LIOBENCH_LIVOX:-$HOME/cs_project/nav/cs/install}"
mkdir -p "$WS/src"
for d in Point-LIO Batch-LIO small_point_lio Super-LIO genz-lio; do
  if [ ! -d "$REFS/$d" ]; then echo "缺少参考算法源码：$REFS/$d" >&2; exit 1; fi
  ln -sfn "$REFS/$d" "$WS/src/$d"
done
# conda 在 PATH 里会让 CMake 链接到 conda 的库
PATH=$(echo "$PATH" | tr ':' '\n' | grep -v -i conda | paste -sd:)
unset CONDA_PREFIX
set +u
source /opt/ros/humble/setup.bash
source "$LIVOX/setup.bash"
set -u
cd "$WS"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
