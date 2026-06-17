#!/usr/bin/env python3
"""
Analyze gimbal disturbance rejection from a ROS bag.

Usage:
    python3 analyze_disturbance.py <bag_file> [--output-dir <dir>]
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
GRAVITY_STILL_S = 2.0       # use first N seconds to estimate gravity
ACCEL_THRESHOLD = 0.5        # m/s^2, below this considered "still"


def load_bag(bag_path, tilt_bias=0.0):
    """Extract command, feedback, and IMU time series.

    tilt_bias: added to raw TILT feedback BEFORE any processing.
    """
    cmd_pan_t, cmd_pan_v = [], []
    cmd_tilt_t, cmd_tilt_v = [], []
    fb_pan_t, fb_pan_v = [], []
    fb_tilt_t, fb_tilt_v = [], []
    imu_t, imu_ax, imu_ay, imu_az = [], [], [], []

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
        elif topic == "/livox/imu":
            imu_t.append(ts)
            imu_ax.append(msg.linear_acceleration.x * 9.81)
            imu_ay.append(msg.linear_acceleration.y * 9.81)
            imu_az.append(msg.linear_acceleration.z * 9.81)
    bag.close()

    missing = []
    if not cmd_pan_t and not cmd_tilt_t: missing.append("/gimbal_cmd")
    if not fb_pan_t: missing.append("/pan")
    if not imu_t: missing.append("/livox/imu")
    if missing:
        print(f"ERROR: No messages found for: {', '.join(missing)}")
        sys.exit(1)

    t0 = min((cmd_pan_t[0] if cmd_pan_t else float("inf")),
             (cmd_tilt_t[0] if cmd_tilt_t else float("inf")),
             fb_pan_t[0] if fb_pan_t else float("inf"),
             imu_t[0] if imu_t else float("inf"))

    def make_ts(v_t, v_v):
        return (np.array(v_t) - t0, np.array(v_v)) if v_t else (np.array([]), np.array([]))

    return {
        "cmd_pan": make_ts(cmd_pan_t, cmd_pan_v),
        "cmd_tilt": make_ts(cmd_tilt_t, cmd_tilt_v),
        "fb_pan": make_ts(fb_pan_t, fb_pan_v),
        "fb_tilt": make_ts(fb_tilt_t, fb_tilt_v),
        "imu": (np.array(imu_t) - t0,
                np.array(imu_ax), np.array(imu_ay), np.array(imu_az)),
    }


def compute_external_accel(data):
    """Subtract estimated gravity vector from raw IMU acceleration."""
    t, ax, ay, az = data["imu"]

    # Estimate gravity from the first still period
    still_mask = t < GRAVITY_STILL_S
    if np.sum(still_mask) < 5:
        print("WARNING: too few IMU samples in still period, using all data for gravity estimate")
        still_mask = np.ones(len(t), dtype=bool)

    g = np.array([np.mean(ax[still_mask]), np.mean(ay[still_mask]), np.mean(az[still_mask])])
    g_norm = np.linalg.norm(g)
    print(f"  Gravity vector: [{g[0]:.3f}, {g[1]:.3f}, {g[2]:.3f}] (|g|={g_norm:.3f} m/s^2)")

    a_ext = np.column_stack([ax - g[0], ay - g[1], az - g[2]])
    a_mag = np.linalg.norm(a_ext, axis=1)
    return t, a_mag, a_ext, g


def sync_feedback_to_imu(data, a_mag_t, a_mag):
    """Interpolate feedback angles onto IMU timestamps for per-sample correlation."""
    out = {}
    for key in ["fb_pan", "fb_tilt"]:
        ft, fv = data[key]
        if len(ft) == 0:
            out[key] = (a_mag_t, np.full_like(a_mag_t, np.nan))
            continue
        out[key] = (a_mag_t, np.interp(a_mag_t, ft, fv))
    return out


def find_disturbance_events(t, a_mag, threshold=ACCEL_THRESHOLD):
    """Find contiguous intervals where |a_ext| exceeds threshold."""
    above = a_mag > threshold
    if not np.any(above):
        return []

    events = []
    in_event = False
    start = 0
    for i in range(len(above)):
        if above[i] and not in_event:
            start = i
            in_event = True
        elif not above[i] and in_event:
            events.append((start, i - 1))
            in_event = False
    if in_event:
        events.append((start, len(above) - 1))

    # Merge events separated by less than 0.5s
    merged = [events[0]]
    for ev in events[1:]:
        gap = t[ev[0]] - t[merged[-1][1]]
        if gap < 0.5:
            merged[-1] = (merged[-1][0], ev[1])
        else:
            merged.append(ev)
    return merged


def analyze(data, out_dir):
    """Run full analysis and generate plots."""
    a_t, a_mag, a_ext, g = compute_external_accel(data)
    fb_synced = sync_feedback_to_imu(data, a_t, a_mag)

    events = find_disturbance_events(a_t, a_mag)
    print(f"  Disturbance events detected: {len(events)}")

    # --- Per-axis analysis ---
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)

    for idx, key in enumerate(["fb_pan", "fb_tilt"]):
        ax = axes[idx]
        label = key.replace("fb_", "").upper()
        ft, fv = fb_synced[key]

        # Angle deviation from commanded position
        if len(data[f"cmd_{label.lower()}"][1]) > 0:
            cmd_interp = np.interp(a_t, data[f"cmd_{label.lower()}"][0],
                                   data[f"cmd_{label.lower()}"][1])
        else:
            cmd_interp = np.zeros_like(a_t)
        deviation = fv - cmd_interp

        color = "tab:blue" if label == "PAN" else "tab:orange"
        ax.plot(a_t, deviation, color=color, linewidth=0.6, label=f"{label} deviation")

        # Highlight disturbance regions
        for start, end in events:
            ax.axvspan(a_t[start], a_t[end], alpha=0.15, color="red")

        ax.axhline(0, color="gray", linestyle="--", linewidth=0.5)
        ax.set_ylabel(f"{label} Deviation (deg)")
        ax.legend(loc="upper right")
        ax.grid(True, alpha=0.3)

        # Per-event metrics
        for i, (start, end) in enumerate(events):
            dev_seg = deviation[start:end + 1]
            if len(dev_seg) == 0:
                continue
            peak_dev = np.max(np.abs(dev_seg))
            peak_accel = np.max(a_mag[start:end + 1])
            print(f"  Event {i+1}: {label} peak_accel={peak_accel:.2f} m/s^2, "
                  f"peak_dev={peak_dev:.3f} deg")

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Disturbance Rejection — Angle Deviation vs External Acceleration")
    fig.tight_layout()

    # --- Top subplot for acceleration ---
    ax_accel = axes[0].twinx()
    ax_accel.plot(a_t, a_mag, color="red", linewidth=0.4, alpha=0.6, label="|a_ext|")
    ax_accel.set_ylabel("|a_ext| (m/s^2)", color="red")
    ax_accel.tick_params(axis="y", labelcolor="red")
    ax_accel.legend(loc="upper left")

    fig.savefig(os.path.join(out_dir, "disturbance_response.png"), dpi=150)
    plt.close(fig)

    # --- Acceleration vs deviation scatter ---
    fig2, axes2 = plt.subplots(1, 2, figsize=(12, 5))
    for idx, key in enumerate(["fb_pan", "fb_tilt"]):
        ax = axes2[idx]
        label = key.replace("fb_", "").upper()
        ft, fv = fb_synced[key]
        if len(data[f"cmd_{label.lower()}"][1]) > 0:
            cmd_interp = np.interp(a_t, data[f"cmd_{label.lower()}"][0],
                                   data[f"cmd_{label.lower()}"][1])
        else:
            cmd_interp = np.zeros_like(a_t)
        deviation = np.abs(fv - cmd_interp)

        # Downsample for scatter plot clarity
        step = max(1, len(a_mag) // 2000)
        ax.scatter(a_mag[::step], deviation[::step], s=2, alpha=0.5)
        ax.set_xlabel("|a_ext| (m/s^2)")
        ax.set_ylabel(f"|{label} Deviation| (deg)")
        ax.set_title(f"{label}")
        ax.grid(True, alpha=0.3)
    fig2.tight_layout()
    fig2.savefig(os.path.join(out_dir, "accel_vs_deviation.png"), dpi=150)
    plt.close(fig2)

    # --- Metrics summary ---
    metrics_path = os.path.join(out_dir, "metrics.txt")
    with open(metrics_path, "w") as f:
        f.write("Disturbance Rejection Metrics\n")
        f.write("============================\n\n")
        for key, label in [("fb_pan", "PAN"), ("fb_tilt", "TILT")]:
            ft, fv = fb_synced[key]
            if len(fv) == 0 or np.all(np.isnan(fv)):
                continue
            if len(data[f"cmd_{label.lower()}"][1]) > 0:
                cmd_interp = np.interp(a_t, data[f"cmd_{label.lower()}"][0],
                                       data[f"cmd_{label.lower()}"][1])
            else:
                cmd_interp = np.zeros_like(a_t)
            deviation = fv - cmd_interp

            max_dev = np.max(np.abs(deviation))
            rms_dev = np.sqrt(np.mean(deviation ** 2))
            max_accel = np.max(a_mag)

            f.write(f"{label}:\n")
            f.write(f"  Max deviation: {max_dev:.3f} deg\n")
            f.write(f"  RMS deviation: {rms_dev:.3f} deg\n")
            f.write(f"  Max external accel: {max_accel:.3f} m/s^2\n")
            f.write("\n")

    print(f"\nPlots and metrics saved to: {out_dir}")


def main():
    parser = argparse.ArgumentParser(description="Analyze gimbal disturbance rejection")
    parser.add_argument("bag", help="Path to ROS bag file")
    parser.add_argument("--output-dir", "-o", default=None, help="Output directory")
    parser.add_argument("--tilt-zero-deg", type=float, default=3.62,
                        help="tilt_zero_deg from pelco_control config (default 3.62)")
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

    print(f"  PAN cmd: {len(data['cmd_pan'][0])} msgs, fb: {len(data['fb_pan'][0])} msgs")
    print(f"  TILT cmd: {len(data['cmd_tilt'][0])} msgs, fb: {len(data['fb_tilt'][0])} msgs")
    print(f"  IMU: {len(data['imu'][0])} msgs")

    analyze(data, out_dir)


if __name__ == "__main__":
    main()
