#!/bin/bash
# Generate score heatmaps (yaw × pitch) for given poses.
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

CONFIG_YAML="$WS_DIR/src/voxel_motion_strategy/configs/voxel_strategy.yaml"
if [ ! -f "$CONFIG_YAML" ]; then
  echo "ERROR: config not found: $CONFIG_YAML"
  exit 1
fi

OUT_DIR="/home/loc/loc_ws/data/rect_viz/score_plot_$$"
mkdir -p "$OUT_DIR"

CSV_FILES=()
POSE_LABELS=()
BEST_LINES=()
IDX=0
while [ $# -ge 3 ]; do
  X="$1"; Y="$2"; Z="$3"; shift 3
  TAG="${X}_${Y}_${Z}"
  CSV="$OUT_DIR/score_${TAG}.csv"
  PPM_FULL="$OUT_DIR/depth_full_${TAG}.ppm"
  PPM_BEST="$OUT_DIR/depth_best_${TAG}.ppm"
  echo "[$IDX] Pose ($X, $Y, $Z) ..."
  BEST=$("$RECT_VIZ" "$OCTOMAP" "$X" "$Y" "$Z" "$CONFIG_YAML" "$CSV" "" "$PPM_FULL" "$PPM_BEST" 2>&1 | grep "Global best")
  echo "  $BEST"
  CSV_FILES+=("$CSV")
  POSE_LABELS+=("($X, $Y, $Z)")
  BEST_LINES+=("$BEST")
  IDX=$((IDX + 1))
done

# Generate 2D heatmap plots
python3 << PYEOF
import csv, os, numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize

csv_files = [$(printf '"%s",' "${CSV_FILES[@]}" | sed 's/,$//')]
labels   = [$(printf '"%s",' "${POSE_LABELS[@]}" | sed 's/,$//')]

n = len(csv_files)
cols = min(n, 2)
rows = (n + cols - 1) // cols
fig, axes = plt.subplots(rows, cols, figsize=(8*cols, 6*rows), squeeze=False)
axes = axes.flatten()

for i, (csv_file, label) in enumerate(zip(csv_files, labels)):
    ax = axes[i]

    # Read full 2D grid
    data = {'yaw_deg': [], 'pitch_deg': [], 'score': [], 'N_eff': []}
    with open(csv_file) as f:
        for row in csv.DictReader(f):
            data['yaw_deg'].append(float(row['yaw_deg']))
            data['pitch_deg'].append(float(row['pitch_deg']))
            data['score'].append(float(row['score']))
            data['N_eff'].append(int(row['N_eff']))

    yaws   = np.array(data['yaw_deg'])
    pitches = np.array(data['pitch_deg'])
    scores  = np.array(data['score'])
    Ns      = np.array(data['N_eff'])

    # Find unique axes
    u_yaws   = np.unique(yaws)
    u_pitches = np.unique(pitches)
    ny, np_ = len(u_yaws), len(u_pitches)

    # Reshape to 2D grid (pitch=rows, yaw=cols)
    heat = np.full((np_, ny), np.nan)
    for j in range(len(scores)):
        yi = np.searchsorted(u_yaws, yaws[j])
        pi = np.searchsorted(u_pitches, pitches[j])
        if yi < ny and pi < np_ and Ns[j] > 0:
            heat[pi, yi] = scores[j]

    # Find global best for annotation
    best_idx = np.nanargmax(heat)
    best_pi, best_yi = np.unravel_index(best_idx, heat.shape)
    best_yaw   = u_yaws[best_yi]
    best_pitch = u_pitches[best_pi]
    best_score = heat[best_pi, best_yi]

    # Heatmap
    vmin = np.nanmin(heat)
    vmax = np.nanmax(heat)
    im = ax.pcolormesh(u_yaws, u_pitches, heat,
                       cmap='jet', shading='auto',
                       norm=Normalize(vmin=vmin, vmax=vmax))

    ax.scatter(best_yaw, best_pitch, marker='*', s=200,
               edgecolors='white', facecolors='none', linewidths=2,
               label=f'best: yaw={best_yaw}° pitch={best_pitch}°\nλ_min={best_score:.0f}')

    ax.set_title(f'Pose {label}')
    ax.set_xlabel('Yaw (°)')
    ax.set_ylabel('Pitch (°)')
    ax.legend(fontsize=8, loc='lower right')
    ax.invert_yaxis()  # pitch up = positive → top of plot
    cbar = plt.colorbar(im, ax=ax, shrink=0.92)
    cbar.set_label('λ_min')

for j in range(n, len(axes)):
    axes[j].set_visible(False)

plt.tight_layout()
out_path = "$OUT_DIR/score_heatmap.png"
plt.savefig(out_path, dpi=120)
print(f"Saved: {out_path}")
PYEOF

echo "CSV files: $OUT_DIR/score_*.csv"
echo "Depth maps: $OUT_DIR/depth_full_*.ppm (full)  $OUT_DIR/depth_best_*.ppm (best-view)"
echo "Plot:       $OUT_DIR/score_heatmap.png"
