#!/usr/bin/env python3
"""
Analyze gimbal step response from a ROS bag.

Usage:
    python3 analyze_step_response.py <bag_file> [--output-dir <dir>]
"""

import argparse
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

try:
    import rosbag
except ImportError:
    print("ERROR: rosbag not found. Source ROS setup.bash first.")
    sys.exit(1)

PAN_CMD = 0x4B
TILT_CMD = 0x4D


def load_bag(bag_path):
    """Extract command and feedback time series from a ROS bag."""
    cmd_pan_t, cmd_pan_v = [], []
    cmd_tilt_t, cmd_tilt_v = [], []
    fb_pan_t, fb_pan_v = [], []
    fb_tilt_t, fb_tilt_v = [], []

    bag = rosbag.Bag(bag_path)
    for topic, msg, t in bag.read_messages():
        ts = t.to_sec()
        if topic == "/gimbal_cmd":
            if msg.cmd == PAN_CMD:
                cmd_pan_t.append(ts)
                cmd_pan_v.append(msg.data)
            elif msg.cmd == TILT_CMD:
                cmd_tilt_t.append(ts)
                cmd_tilt_v.append(msg.data)
        elif topic == "/pan":
            fb_pan_t.append(msg.data[0] if msg.data[0] > 1e9 else ts)
            fb_pan_v.append(msg.data[1])
        elif topic == "/tilt":
            fb_tilt_t.append(msg.data[0] if msg.data[0] > 1e9 else ts)
            fb_tilt_v.append(msg.data[1])
    bag.close()

    if not cmd_pan_t and not cmd_tilt_t:
        print("ERROR: No gimbal commands found in bag.")
        sys.exit(1)

    t0 = min(cmd_pan_t[0] if cmd_pan_t else float("inf"),
             cmd_tilt_t[0] if cmd_tilt_t else float("inf"))

    return {
        "cmd_pan": (np.array(cmd_pan_t) - t0, np.array(cmd_pan_v)),
        "cmd_tilt": (np.array(cmd_tilt_t) - t0, np.array(cmd_tilt_v)),
        "fb_pan": (np.array(fb_pan_t) - t0, np.array(fb_pan_v)),
        "fb_tilt": (np.array(fb_tilt_t) - t0, np.array(fb_tilt_v)),
    }


def detect_steps(t_cmd, v_cmd):
    """Detect step transitions in command signal. Returns list of (start_idx, end_idx, target_value)."""
    steps = []
    diffs = np.diff(v_cmd)
    jump_idx = np.where(np.abs(diffs) > 0.5)[0]
    if len(jump_idx) == 0:
        return steps

    for i, idx in enumerate(jump_idx):
        start = idx + 1  # index after the jump
        end = jump_idx[i + 1] if i + 1 < len(jump_idx) else len(v_cmd) - 1
        target = v_cmd[start]
        steps.append((start, end, target))
    return steps


def compute_step_metrics(t_fb, v_fb, t_cmd, v_cmd, step_info):
    """Compute rise time, settling time, overshoot for one step."""
    start_idx, end_idx, target = step_info
    initial = v_cmd[start_idx - 1]  # value before jump
    amplitude = target - initial
    if abs(amplitude) < 0.1:
        return None

    t0 = t_cmd[start_idx]
    seg_t = t_fb[start_idx:end_idx + 1] - t0
    seg_v = v_fb[start_idx:end_idx + 1]

    if len(seg_t) < 3:
        return None

    # Rise time 10%-90%
    lo = initial + 0.10 * amplitude
    hi = initial + 0.90 * amplitude
    t10 = seg_t[np.where(seg_v >= lo)[0][0]] if np.any(seg_v >= lo) else None
    t90 = seg_t[np.where(seg_v >= hi)[0][0]] if np.any(seg_v >= hi) else None
    rise_time = (t90 - t10) if (t10 is not None and t90 is not None) else None

    # Overshoot
    peak = np.max(seg_v) if amplitude > 0 else np.min(seg_v)
    overshoot_pct = (peak - target) / abs(amplitude) * 100

    # Settling time (2% band)
    band_lo = target - 0.02 * abs(amplitude)
    band_hi = target + 0.02 * abs(amplitude)
    in_band = (seg_v >= band_lo) & (seg_v <= band_hi)
    settled_idx = None
    for j in range(len(in_band) - 1, -1, -1):
        if not in_band[j]:
            settled_idx = j + 1
            break
    settling_time = seg_t[settled_idx] if settled_idx is not None and settled_idx < len(seg_t) else None

    # Steady-state error (last 0.5s of hold)
    last_mask = seg_t >= (seg_t[-1] - 0.5)
    steady_error = np.mean(seg_v[last_mask]) - target if np.any(last_mask) else None

    # Delay (first detectable movement, >2% of amplitude)
    threshold = initial + 0.02 * amplitude
    moved = np.where(np.abs(seg_v - initial) >= 0.02 * abs(amplitude))[0]
    delay = seg_t[moved[0]] if len(moved) > 0 else None

    return {
        "initial": initial, "target": target,
        "rise_time": rise_time, "settling_time": settling_time,
        "overshoot_pct": overshoot_pct, "steady_error": steady_error,
        "delay": delay,
    }


def analyze_axis(ax_name, t_cmd, v_cmd, t_fb, v_fb, out_dir):
    """Analyze one axis and generate plots."""
    steps = detect_steps(t_cmd, v_cmd)
    if not steps:
        print(f"  {ax_name}: no steps detected.")
        return

    metrics_list = []
    for step in steps:
        m = compute_step_metrics(t_fb, v_fb, t_cmd, v_cmd, step)
        if m:
            metrics_list.append(m)

    # --- Plot ---
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t_cmd, v_cmd, "k--", label="Command", linewidth=1)
    ax.plot(t_fb, v_fb, "b-", label="Feedback", linewidth=1)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(f"{ax_name} Angle (deg)")
    ax.set_title(f"{ax_name} Step Response")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, f"step_response_{ax_name.lower()}.png"), dpi=150)
    plt.close(fig)

    return metrics_list


def print_metrics(ax_name, metrics):
    """Print metrics table."""
    print(f"\n  {ax_name} Step Response Metrics:")
    header = f"  {'Step':>6} {'From':>8} {'To':>8} {'Rise(s)':>9} {'Settle(s)':>10} {'Overshoot%':>10} {'Delay(s)':>9} {'SS Err':>8}"
    print(header)
    print("  " + "-" * (len(header) - 2))
    for i, m in enumerate(metrics):
        def fmt(v, w=9):
            return f"{v:>{w}.3f}" if v is not None else f"{'N/A':>{w}}"
        print(f"  {i+1:>6} {m['initial']:>8.1f} {m['target']:>8.1f} "
              f"{fmt(m['rise_time'], 9)} {fmt(m['settling_time'], 10)} "
              f"{fmt(m['overshoot_pct'], 10)} {fmt(m['delay'], 9)} {fmt(m['steady_error'], 8)}")


def main():
    parser = argparse.ArgumentParser(description="Analyze gimbal step response from a ROS bag")
    parser.add_argument("bag", help="Path to ROS bag file")
    parser.add_argument("--output-dir", "-o", default=None, help="Output directory for plots and metrics")
    args = parser.parse_args()

    if not os.path.exists(args.bag):
        print(f"ERROR: Bag file not found: {args.bag}")
        sys.exit(1)

    out_dir = args.output_dir or os.path.splitext(args.bag)[0] + "_analysis"
    os.makedirs(out_dir, exist_ok=True)

    print(f"Loading bag: {args.bag}")
    data = load_bag(args.bag)
    print(f"  PAN cmd: {len(data['cmd_pan'][0])} msgs, fb: {len(data['fb_pan'][0])} msgs")
    print(f"  TILT cmd: {len(data['cmd_tilt'][0])} msgs, fb: {len(data['fb_tilt'][0])} msgs")

    all_metrics = {}
    for ax_name, cmd_key, fb_key in [("PAN", "cmd_pan", "fb_pan"), ("TILT", "cmd_tilt", "fb_tilt")]:
        metrics = analyze_axis(ax_name,
                               data[cmd_key][0], data[cmd_key][1],
                               data[fb_key][0], data[fb_key][1],
                               out_dir)
        if metrics:
            all_metrics[ax_name] = metrics
            print_metrics(ax_name, metrics)

    # Summary file
    summary_path = os.path.join(out_dir, "metrics.txt")
    with open(summary_path, "w") as f:
        def fmt(v, w=9):
            return f"{v:>{w}.3f}" if v is not None else f"{'N/A':>{w}}"
        for ax_name, metrics in all_metrics.items():
            f.write(f"{ax_name} Step Response Metrics:\n")
            f.write(f"{'Step':>6} {'From':>8} {'To':>8} {'Rise(s)':>9} {'Settle(s)':>10} {'Overshoot%':>10} {'Delay(s)':>9} {'SS Err':>8}\n")
            for i, m in enumerate(metrics):
                f.write(f"  {i+1:>6} {m['initial']:>8.1f} {m['target']:>8.1f} "
                        f"{fmt(m['rise_time'], 9)} {fmt(m['settling_time'], 10)} "
                        f"{fmt(m['overshoot_pct'], 10)} {fmt(m['delay'], 9)} {fmt(m['steady_error'], 8)}\n")
            f.write("\n")

    print(f"\nPlots and metrics saved to: {out_dir}")


if __name__ == "__main__":
    main()
