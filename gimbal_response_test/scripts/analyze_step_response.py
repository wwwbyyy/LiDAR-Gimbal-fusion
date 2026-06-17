#!/usr/bin/env python3
"""
Analyze gimbal step response from a ROS bag.

Usage:
    python3 analyze_step_response.py <bag_file> [--output-dir <dir>]

Feedback is sequentially unwrapped to get a continuous curve, then command
values are unwrapped to the same cycle so both signals share the same
continuous angular space.
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


def unwrap_sequential(v):
    """Unwrap [0,360) signal by detecting >180 deg jumps between samples."""
    v = np.asarray(v, dtype=float)
    diff = np.diff(v)
    jumps = np.where(np.abs(diff) > 180.0)[0]
    correction = np.zeros(len(v))
    offset = 0.0
    for j in jumps:
        if diff[j] > 180.0:
            offset -= 360.0
        else:
            offset += 360.0
        correction[j + 1:] = offset
    return v + correction


def unwrap_cmd_to_fb(v_cmd, t_cmd, t_fb, v_fb_unwrapped):
    """Unwrap command values to the same cycle as the (continuous) feedback.

    For each command, find the 360*k offset that puts it closest to the
    interpolated feedback value at that time.
    """
    fb_at_cmd = np.interp(t_cmd, t_fb, v_fb_unwrapped)
    k = np.round((fb_at_cmd - v_cmd) / 360.0)
    return v_cmd + 360.0 * k


def load_bag(bag_path, tilt_bias=0.0):
    """Extract command and feedback time series. Feedback is unwrapped.

    tilt_bias: added to raw TILT feedback BEFORE unwrapping, to compensate
               the known 2*tilt_zero_deg error in pelco_control feedback.
    """
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
            fb_tilt_v.append(msg.data[1] + tilt_bias)
    bag.close()

    if not cmd_pan_t and not cmd_tilt_t:
        print("ERROR: No gimbal commands found in bag.")
        sys.exit(1)

    t0 = min(cmd_pan_t[0] if cmd_pan_t else float("inf"),
             cmd_tilt_t[0] if cmd_tilt_t else float("inf"))

    # Convert to relative time
    cmd_pan_t = np.array(cmd_pan_t) - t0
    cmd_tilt_t = np.array(cmd_tilt_t) - t0
    fb_pan_t = np.array(fb_pan_t) - t0
    fb_tilt_t = np.array(fb_tilt_t) - t0

    # Unwrap feedback for continuity, then unwrap command to same cycle
    fb_pan_uw = unwrap_sequential(np.array(fb_pan_v))
    fb_tilt_uw = unwrap_sequential(np.array(fb_tilt_v))

    cmd_pan_uw = unwrap_cmd_to_fb(np.array(cmd_pan_v), cmd_pan_t,
                                  fb_pan_t, fb_pan_uw) if len(cmd_pan_t) > 0 and len(fb_pan_t) > 0 else np.array(cmd_pan_v)
    cmd_tilt_uw = unwrap_cmd_to_fb(np.array(cmd_tilt_v), cmd_tilt_t,
                                    fb_tilt_t, fb_tilt_uw) if len(cmd_tilt_t) > 0 and len(fb_tilt_t) > 0 else np.array(cmd_tilt_v)

    return {
        "cmd_pan": (cmd_pan_t, cmd_pan_uw),
        "cmd_tilt": (cmd_tilt_t, cmd_tilt_uw),
        "fb_pan": (fb_pan_t, fb_pan_uw),
        "fb_tilt": (fb_tilt_t, fb_tilt_uw),
    }


def detect_steps(t_cmd, v_cmd):
    """Detect step transitions. Returns list of (t_start, t_end, target, initial)."""
    steps = []
    if len(v_cmd) < 2:
        return steps
    diffs = np.abs(np.diff(v_cmd))
    jump_idx = np.where(diffs > 0.5)[0]
    if len(jump_idx) == 0:
        return steps

    for i, idx in enumerate(jump_idx):
        start = idx + 1
        if i + 1 < len(jump_idx):
            t_end = t_cmd[jump_idx[i + 1] + 1]  # start of next step
        else:
            t_end = t_cmd[-1] + 1.0             # last step: use a margin past the end
        steps.append((t_cmd[start], t_end, v_cmd[start], v_cmd[start - 1]))
    return steps


def compute_step_metrics(t_fb, v_fb, t_start, t_end, target, initial):
    """Compute step response metrics by slicing feedback by time."""
    amplitude = target - initial
    if abs(amplitude) < 0.1:
        return None

    mask = (t_fb >= t_start) & (t_fb < t_end)
    seg_t = t_fb[mask] - t_start
    seg_v = v_fb[mask]

    if len(seg_t) < 3:
        return None

    # Rise time 10%-90%
    lo = initial + 0.10 * amplitude
    hi = initial + 0.90 * amplitude
    if amplitude > 0:
        t10 = seg_t[np.where(seg_v >= lo)[0][0]] if np.any(seg_v >= lo) else None
        t90 = seg_t[np.where(seg_v >= hi)[0][0]] if np.any(seg_v >= hi) else None
    else:
        t10 = seg_t[np.where(seg_v <= lo)[0][0]] if np.any(seg_v <= lo) else None
        t90 = seg_t[np.where(seg_v <= hi)[0][0]] if np.any(seg_v <= hi) else None
    rise_time = (t90 - t10) if (t10 is not None and t90 is not None) else None

    # Overshoot
    peak = np.max(seg_v) if amplitude > 0 else np.min(seg_v)
    overshoot_pct = (peak - target) / abs(amplitude) * 100

    # Settling time: first point to enter the 2% band (no overshoot assumed).
    band = 0.02 * abs(amplitude)
    dev = np.abs(seg_v - target)
    in_band = np.where(dev <= band)[0]
    settling_time = seg_t[in_band[0]] if len(in_band) > 0 else None

    # Rise 2%-98%: like rise_time but from 2% to 98% of the step.
    lo2 = initial + 0.02 * amplitude
    hi98 = initial + 0.98 * amplitude
    if amplitude > 0:
        t2 = seg_t[np.where(seg_v >= lo2)[0][0]] if np.any(seg_v >= lo2) else None
        t98 = seg_t[np.where(seg_v >= hi98)[0][0]] if np.any(seg_v >= hi98) else None
    else:
        t2 = seg_t[np.where(seg_v <= lo2)[0][0]] if np.any(seg_v <= lo2) else None
        t98 = seg_t[np.where(seg_v <= hi98)[0][0]] if np.any(seg_v <= hi98) else None
    rise_full = (t98 - t2) if (t2 is not None and t98 is not None) else None

    # Steady-state error (last 0.5s of hold)
    last_mask = seg_t >= (seg_t[-1] - 0.5)
    steady_error = np.mean(seg_v[last_mask]) - target if np.any(last_mask) else None

    # Delay (first detectable movement, >2% of amplitude)
    moved = np.where(np.abs(seg_v - initial) >= 0.02 * abs(amplitude))[0]
    delay = seg_t[moved[0]] if len(moved) > 0 else None

    return {
        "initial": initial, "target": target,
        "amplitude_deg": amplitude,
        "rise_time": rise_time, "rise_full": rise_full,
        "settling_time": settling_time,
        "overshoot_pct": overshoot_pct, "steady_error": steady_error,
        "delay": delay,
    }


def analyze_axis(t_cmd, v_cmd, t_fb, v_fb):
    """Detect steps and compute metrics (no plotting)."""
    steps = detect_steps(t_cmd, v_cmd)
    if not steps:
        return [], None

    metrics_list = []
    for t_start, t_end, target, initial in steps:
        m = compute_step_metrics(t_fb, v_fb, t_start, t_end, target, initial)
        if m:
            metrics_list.append(m)
    return metrics_list, steps


def trim_and_center(t_cmd, v_cmd, t_fb, v_fb, trim=1.5):
    """Trim HOME phase and center angles around zero."""
    pre_cmd = np.where(t_cmd < trim)[0]
    if len(pre_cmd) > 0:
        t_pre, v_pre = t_cmd[pre_cmd[-1]], v_cmd[pre_cmd[-1]]
    else:
        t_pre, v_pre = 0.0, 0.0

    mask_cmd = t_cmd >= trim
    t_cmd = np.concatenate([[t_pre], t_cmd[mask_cmd]])
    v_cmd = np.concatenate([[v_pre], v_cmd[mask_cmd]])
    mask_fb = t_fb >= trim
    t_fb, v_fb = t_fb[mask_fb], v_fb[mask_fb]

    v0 = v_cmd[0]
    v_cmd = v_cmd - v0
    v_fb = v_fb - v0
    return t_cmd, v_cmd, t_fb, v_fb


def plot_paper_figure(data, out_dir):
    """Generate publication-quality step response figures (PDF), one per axis.

    Each figure is trimmed to the axis's active phase and saved separately
    with the same dimensions for side-by-side layout in a paper.
    """
    matplotlib.rcParams.update({
        "font.family": "sans-serif", "font.sans-serif": ["Liberation Sans"],
        "font.size": 11, "axes.labelsize": 12, "legend.fontsize": 10,
        "lines.linewidth": 1.2,
        "text.usetex": False,
    })

    for ax_name, cmd_key, fb_key in [("PAN", "cmd_pan", "fb_pan"),
                                      ("TILT", "cmd_tilt", "fb_tilt")]:
        t_cmd_raw = data[cmd_key][0]
        v_cmd_raw = data[cmd_key][1]
        t_fb_raw = data[fb_key][0]
        v_fb_raw = data[fb_key][1]

        # Trim HOME + center, then further trim to the active steps only
        t_cmd, v_cmd, t_fb, v_fb = trim_and_center(
            t_cmd_raw, v_cmd_raw, t_fb_raw, v_fb_raw)

        # Find active time range: from just before first step to shortly
        # after the last step settles (avoids showing long empty hold time).
        steps = detect_steps(t_cmd, v_cmd)
        if not steps:
            continue
        last_m = compute_step_metrics(t_fb, v_fb,
                                      steps[-1][0], steps[-1][1],
                                      steps[-1][2], steps[-1][3])
        t_min = max(0, steps[0][0] - 0.5)
        if last_m and last_m["settling_time"] is not None:
            t_max = steps[-1][0] + last_m["settling_time"] + 7.0
        else:
            t_max = steps[-1][1] + 0.5

        mask_cmd = (t_cmd >= t_min) & (t_cmd <= t_max)
        mask_fb = (t_fb >= t_min) & (t_fb <= t_max)
        t_cmd_plot = t_cmd[mask_cmd]
        v_cmd_plot = v_cmd[mask_cmd]
        t_fb_plot = t_fb[mask_fb]
        v_fb_plot = v_fb[mask_fb]

        # Prepend the pre-trim command point for the starting horizontal segment
        if len(t_cmd_plot) > 0:
            pre = np.where(t_cmd < t_min)[0]
            if len(pre) > 0:
                t_cmd_plot = np.concatenate([[t_min], t_cmd_plot])
                v_cmd_plot = np.concatenate([[v_cmd[pre[-1]]], v_cmd_plot])

        # Append a dummy point at t_max so the last step has a horizontal segment
        if len(t_cmd_plot) > 0 and t_cmd_plot[-1] < t_max:
            t_cmd_plot = np.concatenate([t_cmd_plot, [t_max]])
            v_cmd_plot = np.concatenate([v_cmd_plot, [v_cmd_plot[-1]]])

        fig, ax = plt.subplots(figsize=(4.5, 3.2))
        ax.step(t_cmd_plot, v_cmd_plot, "k-", label="Command",
                linewidth=1.0, where="post")
        ax.plot(t_fb_plot, v_fb_plot, "b-", label="Feedback", linewidth=1.0)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(f"{ax_name} Angle (deg)")
        ax.set_xlim(t_min, t_max)
        ax.legend(loc="lower right")
        ax.grid(True, alpha=0.15, linestyle="-", linewidth=0.5)

        fig.tight_layout(pad=0.3)
        pdf_path = os.path.join(out_dir, f"step_response_{ax_name.lower()}.pdf")
        fig.savefig(pdf_path, format="pdf", dpi=300, bbox_inches="tight")
        plt.close(fig)
        print(f"  Figure saved: {pdf_path}")


def print_metrics(ax_name, metrics):
    """Print metrics table."""
    print(f"\n  {ax_name} Step Response Metrics:")
    header = (f"  {'Step':>6} {'From':>8} {'To':>8} {'Amp(deg)':>9} "
              f"{'Delay(s)':>9} {'Rise10-90':>10} {'Rise2-98':>10} "
              f"{'Settle(s)':>10} {'Over%':>7}")
    print(header)
    print("  " + "-" * (len(header) - 2))
    for i, m in enumerate(metrics):
        def fmt(v, w=9):
            return f"{v:>{w}.3f}" if v is not None else f"{'N/A':>{w}}"
        print(f"  {i+1:>6} {m['initial']:>8.1f} {m['target']:>8.1f} "
              f"{m['amplitude_deg']:>9.1f} "
              f"{fmt(m['delay'], 9)} {fmt(m['rise_time'], 10)} "
              f"{fmt(m['rise_full'], 10)} {fmt(m['settling_time'], 10)} "
              f"{fmt(m['overshoot_pct'], 7)}")


def main():
    parser = argparse.ArgumentParser(
        description="Analyze gimbal step response from a ROS bag")
    parser.add_argument("bag", help="Path to ROS bag file")
    parser.add_argument("--output-dir", "-o", default=None,
                        help="Output directory for plots and metrics")
    parser.add_argument("--tilt-zero-deg", type=float, default=3.62,
                        help="tilt_zero_deg from pelco_control config (default 3.62). "
                             "TILT feedback has a known 2x bias — set to 0 to disable compensation.")
    args = parser.parse_args()

    if not os.path.exists(args.bag):
        print(f"ERROR: Bag file not found: {args.bag}")
        sys.exit(1)

    out_dir = args.output_dir or os.path.splitext(args.bag)[0] + "_analysis"
    os.makedirs(out_dir, exist_ok=True)

    tilt_bias = 2.0 * args.tilt_zero_deg
    print(f"Loading bag: {args.bag}")
    if tilt_bias != 0:
        print(f"  TILT bias compensation: {tilt_bias:+.2f} deg (2 x tilt_zero_deg={args.tilt_zero_deg})")
    data = load_bag(args.bag, tilt_bias=tilt_bias)
    print(f"  PAN  cmd: {len(data['cmd_pan'][0])} msgs, fb: {len(data['fb_pan'][0])} msgs")
    print(f"  TILT cmd: {len(data['cmd_tilt'][0])} msgs, fb: {len(data['fb_tilt'][0])} msgs")

    all_metrics = {}
    for ax_name, cmd_key, fb_key in [("PAN", "cmd_pan", "fb_pan"),
                                      ("TILT", "cmd_tilt", "fb_tilt")]:
        metrics, _ = analyze_axis(data[cmd_key][0], data[cmd_key][1],
                                  data[fb_key][0], data[fb_key][1])
        if metrics:
            all_metrics[ax_name] = metrics
            print_metrics(ax_name, metrics)

    plot_paper_figure(data, out_dir)

    summary_path = os.path.join(out_dir, "metrics.txt")
    with open(summary_path, "w") as f:
        def fmt(v, w=9):
            return f"{v:>{w}.3f}" if v is not None else f"{'N/A':>{w}}"
        for ax_name, metrics in all_metrics.items():
            f.write(f"{ax_name} Step Response Metrics:\n")
            f.write(f"{'Step':>6} {'From':>8} {'To':>8} {'Amp':>9} "
                    f"{'Rise(s)':>9} {'Settle(s)':>10} {'Overshoot%':>10} "
                    f"{'Delay(s)':>9} {'SS Err':>8}\n")
            for i, m in enumerate(metrics):
                f.write(f"  {i+1:>6} {m['initial']:>8.1f} {m['target']:>8.1f} "
                        f"{m['amplitude_deg']:>9.1f} "
                        f"{fmt(m['rise_time'], 9)} {fmt(m['settling_time'], 10)} "
                        f"{fmt(m['overshoot_pct'], 10)} {fmt(m['delay'], 9)} "
                        f"{fmt(m['steady_error'], 8)}\n")
            f.write("\n")

    print(f"\nPlots and metrics saved to: {out_dir}")


if __name__ == "__main__":
    main()
