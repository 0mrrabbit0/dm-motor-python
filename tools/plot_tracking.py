"""
Collect and plot gripper tracking data.
Runs multiple test moves, saves CSV + PNG plots for analysis.
"""
import csv
import json
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


def collect_move(ctrl, label, from_pct, to_pct, settle_s=1.0, move_dur=None):
    """Move from_pct -> to_pct, log everything."""
    # Go to start position and settle
    ctrl.move_to_pct(from_pct)
    time.sleep(2.0)

    data = {"t": [], "pos": [], "vel": [], "tau": [], "target": [], "q_des": []}

    # Start the move
    target_angle = ctrl._pct_to_rad(to_pct)
    ctrl.move_to_pct(to_pct, duration=move_dur)
    t0 = time.monotonic()

    # Log during move + settle
    total_dur = (move_dur or ctrl.move_duration) + settle_s + 0.5
    while time.monotonic() - t0 < total_dur:
        fb = ctrl._dev.get_feedback(ctrl._mst_id)
        if fb:
            t = time.monotonic() - t0
            data["t"].append(round(t, 4))
            data["pos"].append(fb["q"])
            data["vel"].append(fb["dq"])
            data["tau"].append(fb["tau"])
            data["target"].append(target_angle)
            # Get current trajectory q_des if available
            traj = ctrl._traj
            if traj and not ctrl._traj_done:
                q_des, _, _ = traj.sample()
                data["q_des"].append(q_des)
            else:
                data["q_des"].append(target_angle)
        time.sleep(0.005)

    # Save CSV
    path = os.path.join(OUT_DIR, f"{label}.csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "pos", "vel", "tau", "target", "q_des"])
        for i in range(len(data["t"])):
            w.writerow([data["t"][i], data["pos"][i], data["vel"][i],
                        data["tau"][i], data["target"][i], data["q_des"][i]])
    print(f"  [{label}] {len(data['t'])} samples -> {path}")
    return data


def plot_data(data, label):
    """Plot tracking data and save as PNG."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("  matplotlib not available, skipping plot")
        return

    t = data["t"]
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle(f"Gripper Tracking: {label}", fontsize=14)

    # Position
    ax = axes[0]
    ax.plot(t, data["pos"], 'b-', linewidth=0.8, label='actual pos')
    ax.plot(t, data["q_des"], 'g--', linewidth=1.0, label='q_des (trajectory)')
    ax.plot(t, data["target"], 'r:', linewidth=1.0, label='final target')
    ax.set_ylabel("Position (rad)")
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    # Position error
    error = [data["pos"][i] - data["q_des"][i] for i in range(len(t))]
    ax2 = ax.twinx()
    ax2.plot(t, error, 'orange', linewidth=0.5, alpha=0.6, label='tracking error')
    ax2.set_ylabel("Error (rad)", color='orange')
    ax2.tick_params(axis='y', labelcolor='orange')

    # Velocity
    ax = axes[1]
    ax.plot(t, data["vel"], 'b-', linewidth=0.8)
    ax.set_ylabel("Velocity (rad/s)")
    ax.axhline(y=0, color='k', linewidth=0.3)
    ax.grid(True, alpha=0.3)

    # Torque
    ax = axes[2]
    ax.plot(t, data["tau"], 'r-', linewidth=0.8)
    ax.set_ylabel("Torque (Nm)")
    ax.set_xlabel("Time (s)")
    ax.axhline(y=0, color='k', linewidth=0.3)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    path = os.path.join(OUT_DIR, f"{label}.png")
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"  Plot saved: {path}")
    return path


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

        ctrl.load_friction()
        ctrl.enable()
        time.sleep(0.5)

        plots = []

        # Test 1: Small move (should be smooth)
        print("\n[TEST 1] Small move: 40% -> 60%")
        d = collect_move(ctrl, "small_40_60", 40, 60)
        p = plot_data(d, "small_40_60")
        if p: plots.append(p)

        # Test 2: Large move (the problem case)
        print("\n[TEST 2] Large move: 10% -> 90%")
        d = collect_move(ctrl, "large_10_90", 10, 90)
        p = plot_data(d, "large_10_90")
        if p: plots.append(p)

        # Test 3: Full travel
        print("\n[TEST 3] Full travel: 0% -> 100%")
        d = collect_move(ctrl, "full_0_100", 0, 100)
        p = plot_data(d, "full_0_100")
        if p: plots.append(p)

        # Test 4: Large move reverse
        print("\n[TEST 4] Large reverse: 90% -> 10%")
        d = collect_move(ctrl, "large_90_10", 90, 10)
        p = plot_data(d, "large_90_10")
        if p: plots.append(p)

        print(f"\n=== Done. Plots saved to {OUT_DIR}/ ===")
        for p in plots:
            print(f"  {p}")

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
