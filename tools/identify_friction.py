"""
Safe friction parameter identification for the lead screw gripper.

Uses the GripperController's smooth trajectory moves (already validated)
to extract friction parameters from position/velocity/torque data.

NO direct velocity control — avoids overcurrent/stall issues entirely.

Method:
  1. Run multiple moves at different speeds (by varying trajectory duration)
  2. During steady-state motion, measure: velocity and torque
  3. At constant velocity: tau_applied ≈ friction(v)
  4. Fit Stribeck curve to (velocity, friction) data points

Usage:
    LD_LIBRARY_PATH=... PYTHONPATH=src python3 tools/identify_friction.py
"""
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
FRICTION_PATH = os.path.join(os.path.dirname(__file__), "..", "gripper_friction.json")
OUT_DIR = os.path.join(os.path.dirname(__file__), "..", "diag_data")


def startup_recover(dev, max_retries=3):
    """Check if motor is in error state on startup and recover via CAN bus reset."""
    # First try: just enable normally
    for _ in range(5):
        dev.enable(CAN_ID)
        time.sleep(0.005)
    time.sleep(0.2)
    for _ in range(20):
        dev.control_mit(CAN_ID, kp=0, kd=1.0, q=0, dq=0, tau=0)
        time.sleep(0.005)
    fb = dev.get_feedback(MST_ID)
    if fb and fb["err"] == 1:
        print("[MOTOR] OK (err=1, enabled)")
        return True

    # Motor in error state — reset CAN bus with increasing pause
    for attempt in range(max_retries):
        pause = 3.0 + attempt * 2.0  # 3s, 5s, 7s
        print(f"[RECOVER] Attempt {attempt+1}/{max_retries}: "
              f"resetting CAN bus (pause {pause:.0f}s)...")
        dev.reset(pause_s=pause)

        # Try enable after reset
        for _ in range(5):
            dev.enable(CAN_ID)
            time.sleep(0.01)
        time.sleep(0.3)
        for _ in range(50):
            dev.control_mit(CAN_ID, kp=0, kd=1.0, q=0, dq=0, tau=0)
            time.sleep(0.005)
        fb = dev.get_feedback(MST_ID)
        if fb:
            print(f"  Feedback: err={fb['err']}")
            if fb["err"] == 1:
                print("[RECOVER] Motor recovered!")
                return True
            elif fb["err"] == 0:
                # Disabled but not error — just needs enable
                for _ in range(5):
                    dev.enable(CAN_ID)
                    time.sleep(0.01)
                time.sleep(0.2)
                fb2 = dev.get_feedback(MST_ID)
                if fb2 and fb2["err"] == 1:
                    print("[RECOVER] Motor recovered!")
                    return True
        else:
            print("  No feedback received")

    print("[RECOVER] FAILED after all retries. Power cycle the motor.")
    return False


def collect_move_data(ctrl, from_pct, to_pct, duration):
    """Move and collect raw feedback during motion."""
    ctrl.move_to_pct(from_pct)
    time.sleep(max(duration + 0.5, 2.0))

    data = []
    ctrl.move_to_pct(to_pct, duration=duration)
    t0 = time.monotonic()

    while time.monotonic() - t0 < duration + 0.5:
        fb = ctrl._dev.get_feedback(ctrl._mst_id)
        if fb and fb["err"] == 1:
            data.append({
                "t": time.monotonic() - t0,
                "vel": fb["dq"],
                "tau": fb["tau"],
            })
        time.sleep(0.005)

    return data


def extract_friction_points(data, duration):
    """Extract (velocity, friction_torque) points from move data.

    During steady-state motion (middle 60% of trajectory),
    the net torque ≈ friction because acceleration ≈ 0.
    """
    points = []
    # Use middle 20-80% of trajectory (steady velocity phase)
    t_start = duration * 0.2
    t_end = duration * 0.8

    mid_data = [d for d in data if t_start <= d["t"] <= t_end]
    if len(mid_data) < 5:
        return points

    # Average over small windows to reduce noise
    window = max(3, len(mid_data) // 5)
    for i in range(0, len(mid_data) - window, window):
        chunk = mid_data[i:i + window]
        avg_vel = sum(d["vel"] for d in chunk) / len(chunk)
        avg_tau = sum(d["tau"] for d in chunk) / len(chunk)
        if abs(avg_vel) > 0.1:  # skip near-zero velocity
            points.append((avg_vel, avg_tau))

    return points


def fit_stribeck(points):
    """Fit Stribeck parameters from (velocity, torque) data.

    F(v) = sign(v) * [Fc + (Fs - Fc) * exp(-(v/vs)²)] + Fv * v
    """
    abs_vel = [abs(v) for v, _ in points]
    abs_tau = [abs(t) for _, t in points]

    if len(abs_vel) < 3:
        return None

    # Initial estimates
    tau_sorted = sorted(zip(abs_vel, abs_tau))

    # Fv from high-speed slope
    high = [(v, t) for v, t in tau_sorted if v > max(abs_vel) * 0.5]
    low = [(v, t) for v, t in tau_sorted if v < max(abs_vel) * 0.3]

    if len(high) >= 2:
        fv_est = max(0.001, (high[-1][1] - high[0][1]) / max(0.01, high[-1][0] - high[0][0]))
    else:
        fv_est = 0.01

    fc_est = max(0.01, min(abs_tau) if abs_tau else 0.1)
    fs_est = max(fc_est * 1.05, max(abs_tau[:len(abs_tau)//3]) if abs_tau else 0.15)

    # Grid search
    best_err = float('inf')
    best = (fc_est, fs_est, 2.0, fv_est)

    for fc_m in [0.6, 0.8, 1.0, 1.2, 1.5]:
        for fs_m in [0.8, 1.0, 1.2, 1.5]:
            for vs in [0.3, 0.5, 1.0, 2.0, 3.0, 5.0, 8.0]:
                for fv_m in [0.3, 0.6, 1.0, 1.5, 2.0]:
                    fc = fc_est * fc_m
                    fs = max(fc * 1.01, fs_est * fs_m)
                    fv = fv_est * fv_m
                    err = sum(
                        (fc + (fs - fc) * math.exp(-(v / vs)**2) + fv * v - t)**2
                        for v, t in zip(abs_vel, abs_tau)
                    )
                    if err < best_err:
                        best_err = err
                        best = (fc, fs, vs, fv)

    return best


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    dev = DmDevice()
    ctrl = None

    try:
        dev.open(nom_baud_hz=1_000_000, dat_baud_hz=5_000_000, sn=ADAPTER_SN)

        # Auto-recover if motor is in error state
        if not startup_recover(dev):
            return

        ctrl = GripperController(dev, CAN_ID, MST_ID)

        if os.path.exists(CALIB_PATH):
            ctrl.load_calibration(CALIB_PATH)
        else:
            print("ERROR: No calibration. Run gripper_demo.py first.")
            return

        ctrl.enable()
        time.sleep(0.5)

        # Run moves at different speeds by varying duration
        # 30%→70% = 40% travel, different durations → different velocities
        test_configs = [
            (30, 70, 3.0, "very slow"),
            (30, 70, 2.0, "slow"),
            (30, 70, 1.2, "medium"),
            (30, 70, 0.8, "fast"),
            (30, 70, 0.5, "very fast"),
            # Reverse direction
            (70, 30, 3.0, "rev slow"),
            (70, 30, 1.2, "rev medium"),
            (70, 30, 0.5, "rev fast"),
        ]

        all_points = []

        print(f"\n{'test':>12s} | {'dur':>5s} | {'n_pts':>5s} | {'avg_vel':>8s} | {'avg_tau':>8s}")
        print("-" * 55)

        for from_pct, to_pct, dur, label in test_configs:
            data = collect_move_data(ctrl, from_pct, to_pct, dur)
            points = extract_friction_points(data, dur)

            if points:
                avg_v = sum(abs(v) for v, _ in points) / len(points)
                avg_t = sum(abs(t) for _, t in points) / len(points)
                all_points.extend(points)
                print(f"{label:>12s} | {dur:5.1f} | {len(points):5d} | {avg_v:8.2f} | {avg_t:8.4f}")
            else:
                print(f"{label:>12s} | {dur:5.1f} | {'none':>5s} |")

        if len(all_points) < 5:
            print("\nERROR: Not enough data points.")
            return

        # Fit Stribeck curve
        result = fit_stribeck(all_points)
        if result is None:
            print("\nERROR: Fitting failed.")
            return

        fc, fs, vs, fv = result
        sigma0 = 1000.0
        sigma1 = min(1.0, fc * 2)

        print(f"\n=== Identified Friction Parameters ===")
        print(f"  Fc (Coulomb):   {fc:.4f} Nm")
        print(f"  Fs (Static):    {fs:.4f} Nm")
        print(f"  vs (Stribeck):  {vs:.2f} rad/s")
        print(f"  σ0 (stiffness): {sigma0:.0f} Nm/rad")
        print(f"  σ1 (damping):   {sigma1:.1f} Nm·s/rad")
        print(f"  Fv (viscous):   {fv:.5f} Nm·s/rad")
        print(f"  Fs/Fc ratio:    {fs/fc:.2f}")

        # Save
        params = {"fc": fc, "fs": fs, "vs": vs,
                  "sigma0": sigma0, "sigma1": sigma1, "fv": fv}
        with open(FRICTION_PATH, "w") as f:
            json.dump(params, f, indent=2)
        print(f"\nSaved to {FRICTION_PATH}")

        # Plot
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt

            fig, ax = plt.subplots(figsize=(10, 6))

            v_pts = [abs(v) for v, _ in all_points]
            t_pts = [abs(t) for _, t in all_points]
            ax.scatter(v_pts, t_pts, c='blue', s=20, alpha=0.5, label='Measured')

            v_plot = [i * 0.05 for i in range(1, 400)]
            f_plot = [fc + (fs - fc) * math.exp(-(v / vs)**2) + fv * v for v in v_plot]
            ax.plot(v_plot, f_plot, 'r-', linewidth=2, label=f'Fit: Fc={fc:.3f} Fs={fs:.3f} vs={vs:.1f}')

            ax.set_xlabel("Velocity |v| (rad/s)")
            ax.set_ylabel("Friction |τ| (Nm)")
            ax.set_title("Stribeck Curve Identification")
            ax.legend()
            ax.grid(True, alpha=0.3)

            path = os.path.join(OUT_DIR, "stribeck_curve.png")
            plt.savefig(path, dpi=150)
            plt.close()
            print(f"Plot: {path}")
        except ImportError:
            pass

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
