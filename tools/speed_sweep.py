"""
Speed sweep: test progressively faster trajectories to find the
maximum smooth speed for the lead screw gripper.

Runs 0%→80% moves at different durations, measures velocity ripple
and position tracking error. Outputs a summary table + plots.
"""
import csv
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from dm_motor import DmDevice, GripperController

ADAPTER_SN = "14AA044B241402B10DDBDAFE448040BB"
CAN_ID = 0x01
MST_ID = 0x11
CALIB_PATH = os.path.join(os.path.dirname(__file__), "..", "gripper_calibration.json")
OUT_DIR = os.path.join(os.path.dirname(__file__), "..", "diag_data")

FROM_PCT = 20.0
TO_PCT = 80.0


def run_move(ctrl, duration, settle=1.5):
    """Execute one move and return data + metrics."""
    ctrl.move_to_pct(FROM_PCT)
    time.sleep(2.0)

    target_angle = ctrl._pct_to_rad(TO_PCT)
    data = {"t": [], "pos": [], "vel": [], "tau": []}

    ctrl.move_to_pct(TO_PCT, duration=duration)
    t0 = time.monotonic()

    while time.monotonic() - t0 < duration + settle:
        fb = ctrl._dev.get_feedback(ctrl._mst_id)
        if fb:
            data["t"].append(round(time.monotonic() - t0, 4))
            data["pos"].append(fb["q"])
            data["vel"].append(fb["dq"])
            data["tau"].append(fb["tau"])
        time.sleep(0.005)

    # Metrics: velocity ripple during motion phase (10%-90% of duration)
    n = len(data["t"])
    i_start = next((i for i in range(n) if data["t"][i] >= duration * 0.1), 0)
    i_end = next((i for i in range(n) if data["t"][i] >= duration * 0.9), n - 1)
    motion_vel = data["vel"][i_start:i_end]
    motion_tau = data["tau"][i_start:i_end]

    if len(motion_vel) < 10:
        return data, {"error": "insufficient data"}

    # Compute smoothed velocity trend (moving average window=5)
    w = 5
    trend = []
    for i in range(len(motion_vel)):
        lo = max(0, i - w)
        hi = min(len(motion_vel), i + w + 1)
        trend.append(sum(motion_vel[lo:hi]) / (hi - lo))

    # Ripple = RMS of (velocity - trend)
    ripple = [motion_vel[i] - trend[i] for i in range(len(motion_vel))]
    vel_ripple_rms = math.sqrt(sum(r ** 2 for r in ripple) / len(ripple))
    vel_ripple_p2p = max(ripple) - min(ripple)

    # Peak velocity
    vel_peak = max(abs(v) for v in motion_vel)

    # Tau ripple
    tau_mean = sum(motion_tau) / len(motion_tau)
    tau_ripple = [t - tau_mean for t in motion_tau]
    tau_ripple_rms = math.sqrt(sum(r ** 2 for r in tau_ripple) / len(tau_ripple))

    # Settling: time after trajectory for pos to stay within 0.01 rad of target
    settle_data = [(data["t"][i], data["pos"][i]) for i in range(n)
                   if data["t"][i] >= duration]
    settle_time = 0
    for t_s, p_s in reversed(settle_data):
        if abs(p_s - target_angle) > 0.05:
            settle_time = t_s - duration
            break

    return data, {
        "duration": duration,
        "vel_peak": round(vel_peak, 2),
        "vel_ripple_rms": round(vel_ripple_rms, 3),
        "vel_ripple_p2p": round(vel_ripple_p2p, 3),
        "tau_ripple_rms": round(tau_ripple_rms, 3),
        "settle_time": round(settle_time, 3),
    }


def plot_comparison(all_data, durations):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("  matplotlib not available")
        return

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=False)
    fig.suptitle("Speed Sweep: Velocity Smoothness vs Duration", fontsize=14)
    colors = plt.cm.viridis([i / max(1, len(durations) - 1) for i in range(len(durations))])

    for idx, (dur, data) in enumerate(zip(durations, all_data)):
        t = data["t"]
        # Normalize time to trajectory progress
        t_norm = [ti / dur for ti in t]
        label = f"T={dur:.1f}s"
        c = colors[idx]

        axes[0].plot(t, data["pos"], color=c, linewidth=0.8, label=label)
        axes[1].plot(t, data["vel"], color=c, linewidth=0.8, label=label)
        axes[2].plot(t, data["tau"], color=c, linewidth=0.8, label=label)

    axes[0].set_ylabel("Position (rad)")
    axes[0].legend(fontsize=7, loc='upper right')
    axes[0].grid(True, alpha=0.3)
    axes[1].set_ylabel("Velocity (rad/s)")
    axes[1].grid(True, alpha=0.3)
    axes[2].set_ylabel("Torque (Nm)")
    axes[2].set_xlabel("Time (s)")
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    path = os.path.join(OUT_DIR, "speed_sweep.png")
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"  Plot: {path}")


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    dev = DmDevice()
    ctrl = None

    try:
        dev.open(nom_baud_hz=1_000_000, dat_baud_hz=5_000_000, sn=ADAPTER_SN)
        ctrl = GripperController(dev, CAN_ID, MST_ID)

        if os.path.exists(CALIB_PATH):
            ctrl.load_calibration(CALIB_PATH)
        else:
            print("No calibration. Run gripper_demo.py first.")
            return

        ctrl.enable()
        time.sleep(0.5)

        # Sweep durations from slow to fast
        # 20%→80% = 60% of travel ≈ 6.6 rad
        durations = [2.0, 1.5, 1.2, 1.0, 0.8, 0.6, 0.5, 0.4]
        results = []
        all_data = []

        print(f"\n{'dur(s)':>7s} | {'v_peak':>7s} | {'v_rip_rms':>9s} | {'v_rip_p2p':>9s} | {'tau_rip':>7s} | {'settle':>7s} | Grade")
        print("-" * 75)

        for dur in durations:
            data, m = run_move(ctrl, dur)
            all_data.append(data)
            results.append(m)

            if "error" in m:
                print(f"{dur:7.1f} | {'error':>7s}")
                continue

            # Grade: smooth if vel_ripple_rms < 0.5 and p2p < 2.0
            smooth = m["vel_ripple_rms"] < 0.5 and m["vel_ripple_p2p"] < 2.0
            grade = "SMOOTH" if smooth else "RIPPLE"

            print(f"{dur:7.1f} | {m['vel_peak']:7.1f} | {m['vel_ripple_rms']:9.3f} | "
                  f"{m['vel_ripple_p2p']:9.3f} | {m['tau_ripple_rms']:7.3f} | "
                  f"{m['settle_time']:7.3f} | {grade}")

        # Find fastest smooth duration
        smooth_durs = [r["duration"] for r in results
                       if "error" not in r
                       and r["vel_ripple_rms"] < 0.5
                       and r["vel_ripple_p2p"] < 2.0]
        if smooth_durs:
            fastest = min(smooth_durs)
            print(f"\n=== Fastest smooth duration: {fastest:.1f}s "
                  f"(peak vel: {next(r['vel_peak'] for r in results if r.get('duration')==fastest):.1f} rad/s) ===")
        else:
            print("\n=== No smooth duration found in range ===")

        plot_comparison(all_data, durations)

        # Save results
        path = os.path.join(OUT_DIR, "speed_sweep.json")
        with open(path, "w") as f:
            json.dump(results, f, indent=2)
        print(f"Results: {path}")

    finally:
        if ctrl:
            try:
                ctrl.disable()
            except Exception:
                pass
        dev.close()
        print("[DONE]")


if __name__ == "__main__":
    main()
