#!/bin/bash
# Generate score-vs-yaw plots for given poses.
# Usage: ./score_plot.sh <octomap_prefix> x1 y1 z1 [x2 y2 z2 ...]
#
# Example:
#   ./score_plot.sh /path/to/octomap 261 -362 2 330 -84 2

set -e

OCTOMAP="$1"
shift

if [ $# -lt 3 ]; then
  echo "Usage: $0 <octomap_prefix> x1 y1 z1 [x2 y2 z2 ...]"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
BUILD_BIN="$WS_DIR/build/voxel_motion_strategy/rect_viz"
DEVEL_BIN="$WS_DIR/devel/lib/voxel_motion_strategy/rect_viz"

RECT_VIZ="${DEVEL_BIN}"
if [ ! -x "$RECT_VIZ" ]; then
  RECT_VIZ="${BUILD_BIN}"
fi
if [ ! -x "$RECT_VIZ" ]; then
  echo "ERROR: rect_viz not found. Build voxel_motion_strategy first."
  exit 1
fi

OUT_DIR="/tmp/score_plot_$$"
mkdir -p "$OUT_DIR"

POSE_ARGS=()
POSE_LABELS=()
IDX=0
while [ $# -ge 3 ]; do
  X="$1"; Y="$2"; Z="$3"; shift 3
  TAG="${X}_${Y}_${Z}"
  CSV="$OUT_DIR/score_${TAG}.csv"
  echo "[$IDX] Pose ($X, $Y, $Z) ..."
  "$RECT_VIZ" "$OCTOMAP" "$X" "$Y" "$Z" "$CSV" 2>&1 | grep "Global best"
  POSE_ARGS+=("$CSV")
  POSE_LABELS+=("($X, $Y, $Z)")
  IDX=$((IDX + 1))
done

# Generate plot
python3 << PYEOF
import csv, os, numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

csv_files = [$(printf '"%s",' "${POSE_ARGS[@]}" | sed 's/,$//')]
labels   = [$(printf '"%s",' "${POSE_LABELS[@]}" | sed 's/,$//')]

n = len(csv_files)
cols = min(n, 3)
rows = (n + cols - 1) // cols
fig, axes = plt.subplots(rows, cols, figsize=(6*cols, 5*rows), squeeze=False)
axes = axes.flatten()

for i, (csv_file, label) in enumerate(zip(csv_files, labels)):
    ax = axes[i]
    yaws, scores, pitches = [], [], []
    with open(csv_file) as f:
        for row in csv.DictReader(f):
            yaws.append(float(row['yaw_deg']))
            scores.append(float(row['score']))
            pitches.append(float(row['best_pitch_deg']))
    best_idx = np.argmax(scores)
    best_yaw, best_score, best_pitch = yaws[best_idx], scores[best_idx], pitches[best_idx]

    ax.plot(yaws, scores, 'b-', linewidth=1, alpha=0.8)
    ax.axvline(best_yaw, color='r', linestyle='--', alpha=0.5,
               label=f'best: yaw={best_yaw}° pitch={best_pitch}° λ_min={best_score:.0f}')
    ax.set_title(f'Pose {label}')
    ax.set_xlabel('Yaw (°)')
    ax.set_ylabel('λ_min')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

for j in range(n, len(axes)):
    axes[j].set_visible(False)

plt.tight_layout()
out_path = "$OUT_DIR/score_vs_yaw.png"
plt.savefig(out_path, dpi=120)
print(f"Saved: {out_path}")
PYEOF

echo "CSV files: $OUT_DIR/score_*.csv"
echo "Plot:      $OUT_DIR/score_vs_yaw.png"
