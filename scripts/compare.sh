#!/usr/bin/env bash
# Usage: compare.sh <reference.tum> <estimate.tum> [more.tum ...]
# The reference can be ground truth or another LIO run (then APE measures agreement, not error).
set -euo pipefail
command -v evo_ape >/dev/null || { echo "evo not installed: pip install evo"; exit 1; }
[ $# -ge 2 ] || { echo "usage: $0 <ref.tum> <est.tum> [...]"; exit 1; }
ref="$1"; shift
for est in "$@"; do
  echo "== APE (aligned) $est vs $ref"
  evo_ape tum "$ref" "$est" -a --t_max_diff 0.02
done
evo_traj tum "$@" --ref "$ref" -a --plot_mode xy --save_plot traj_compare.pdf
echo "plot saved to traj_compare.pdf"
