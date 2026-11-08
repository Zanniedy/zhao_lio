#!/usr/bin/env bash
# 复现 HANDOFF-backend.md 第 6 节的全部实验。结果写到 $OUT（默认 /tmp/zb）。
# 需要：已编译的工作空间，以及 liobench 的 venv（里面有 evo、scipy）。
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
OUT=${OUT:-/tmp/zb}
PY=$ROOT/liobench/.venv/bin/python
EVO=$ROOT/liobench/.venv/bin
T01=$HOME/datasets/liobench/narrowwide/tracked-01/bag
GT=$HOME/datasets/liobench/narrowwide/tracked-01/gt.tum
OWN=$HOME/datasets/liobench/own/offline_map_20261014/bag
CT=$ROOT/scripts/backend/replay_tracked01.yaml
CO=$ROOT/scripts/backend/replay_own_mid360.yaml
set +u; source /opt/ros/humble/setup.bash; source ~/cs_project/nav/cs/install/setup.bash; source "$ROOT/install/setup.bash"; set -u
mkdir -p "$OUT"/{t01,t01loc,own}
R() { ros2 run zhao_lio_ros2 zhao_lio_replay --ros-args "$@"; }
summary() { grep '^backend' "$1"; }

# tracked-01：建图 + 回环
R --params-file "$CT" -p bag:="$T01" -p traj_path:="$OUT/t01/lio.tum" -p backend.mode:=mapping \
  -p backend.traj_path:="$OUT/t01/rt.tum" -p backend.opt_traj_path:="$OUT/t01/opt.tum" \
  -p backend.save_dir:="$OUT/t01/map" > "$OUT/t01/log.txt" 2>&1; summary "$OUT/t01/log.txt"
# tracked-01：从 200 s 开始先验图定位（不给初值），以及两种注入
L() { R --params-file "$CT" -p bag:="$T01" -p start_offset:=200.0 -p backend.mode:=localization -p backend.map_path:="$OUT/t01/map" "$@"; }
L -p traj_path:="$OUT/t01loc/lio.tum" -p backend.traj_path:="$OUT/t01loc/rt.tum" > "$OUT/t01loc/log.txt" 2>&1; summary "$OUT/t01loc/log.txt"
L -p backend.traj_path:="$OUT/t01loc/ramp.tum" -p test_perturb_time:=350.0 -p test_perturb_ramp:=10.0 -p "test_perturb:=[3.0, 2.0, 25.0]" > "$OUT/t01loc/log_ramp.txt" 2>&1; summary "$OUT/t01loc/log_ramp.txt"
L -p backend.traj_path:="$OUT/t01loc/div.tum" -p test_perturb_time:=350.0 -p "test_perturb:=[30.0, 0.0, 90.0]" > "$OUT/t01loc/log_div.txt" 2>&1; summary "$OUT/t01loc/log_div.txt"
# 自录 Mid-360：建图，再从 60 s 开始定位
R --params-file "$CO" -p bag:="$OWN" -p traj_path:="$OUT/own/lio.tum" -p backend.mode:=mapping \
  -p backend.traj_path:="$OUT/own/rt.tum" -p backend.opt_traj_path:="$OUT/own/opt.tum" \
  -p backend.save_dir:="$OUT/own/map" > "$OUT/own/log.txt" 2>&1; summary "$OUT/own/log.txt"
O() { R --params-file "$CO" -p bag:="$OWN" -p start_offset:=60.0 -p backend.mode:=localization -p backend.map_path:="$OUT/own/map" "$@"; }
O -p traj_path:="$OUT/own/loc_lio.tum" -p backend.traj_path:="$OUT/own/loc.tum" > "$OUT/own/loc_log.txt" 2>&1; summary "$OUT/own/loc_log.txt"
O -p backend.traj_path:="$OUT/own/loc_ramp.tum" -p test_perturb_time:=100.0 -p test_perturb_ramp:=8.0 -p "test_perturb:=[2.0, 2.0, 20.0]" > "$OUT/own/loc_ramp_log.txt" 2>&1; summary "$OUT/own/loc_ramp_log.txt"
O -p backend.traj_path:="$OUT/own/loc_div.tum" -p test_perturb_time:=100.0 -p "test_perturb:=[20.0, 0.0, 90.0]" > "$OUT/own/loc_div_log.txt" 2>&1; summary "$OUT/own/loc_div_log.txt"

echo "== tracked-01 建图（evo，SE(3) 对齐）"
for f in lio rt opt; do
  printf "%-4s ATE " $f; "$EVO/evo_ape" tum "$GT" "$OUT/t01/$f.tum" -a 2>/dev/null | awk '/rmse/{printf "%s  ",$2}/max/{printf "max %s  ",$2}'
  printf "RPE200m "; "$EVO/evo_rpe" tum "$GT" "$OUT/t01/$f.tum" --delta 200 --delta_unit m --all_pairs 2>/dev/null | awk '/rmse/{print $2}'
done
echo "== tracked-01 定位（map 系误差）"
$PY "$ROOT/scripts/backend/lio_in_map.py" "$OUT/t01loc/rt.tum" "$OUT/t01loc/lio.tum" "$OUT/t01loc/lio_map.tum"
$PY "$ROOT/scripts/backend/loc_eval.py" "$GT" "$OUT/t01/opt.tum" "$OUT/t01loc/lio_map.tum" "$OUT/t01loc/rt.tum" "$OUT/t01loc/ramp.tum" "$OUT/t01loc/div.tum" | grep -v bins
echo "== 自录数据定位（以建图轨迹为参考，没有真值）"
$PY "$ROOT/scripts/backend/lio_in_map.py" "$OUT/own/loc.tum" "$OUT/own/loc_lio.tum" "$OUT/own/loc_lio_map.tum"
$PY - "$OUT/own" <<'PY'
import sys, numpy as np
from scipy.spatial.transform import Rotation as Rot
d=sys.argv[1]; ref=np.loadtxt(d+'/opt.tum')
for n in ['loc_lio_map','loc','loc_ramp','loc_div']:
    e=np.loadtxt('%s/%s.tum'%(d,n)); i=np.clip(np.searchsorted(ref[:,0],e[:,0]),0,len(ref)-1)
    ok=np.abs(ref[i,0]-e[:,0])<0.03; r=ref[i[ok]]; e=e[ok]; p=np.linalg.norm(e[:,1:4]-r[:,1:4],axis=1)
    a=np.degrees((Rot.from_quat(r[:,4:8]).inv()*Rot.from_quat(e[:,4:8])).magnitude())
    print("%-12s pos rmse %.3f max %.3f | rot max %.2f deg"%(n,np.sqrt((p**2).mean()),p.max(),a.max()))
PY
