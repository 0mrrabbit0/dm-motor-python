"""
Gripper auto-tuning tool.

1. Logs step response data (pos, vel, tau vs time)
2. Analyzes oscillation (frequency, amplitude, overshoot, settling time)
3. Iteratively adjusts Kp/Kd until oscillation is eliminated
4. Saves optimal parameters

Usage:
    LD_LIBRARY_PATH=... PYTHONPATH=src python3 tools/tune_gripper.py
"""
import json
import os
import sys
import time
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from dm_motor import DmDevice, GripperController

ADAPTER_SN = "14AA044B241402B10DDBDAFE448040BB"
CAN_ID = 0x01
MST_ID = 0x11
CALIB_PATH = os.path.join(os.path.dirname(__file__), "..", "gripper_calibration.json")


def collect_step_response(ctrl: GripperController, target_pct: float,
                          duration_s: float = 3.0) -> dict:
    """Execute a step move and record response data."""
    data = {"t": [], "pos": [], "vel": [], "tau": [], "target": [], "err": []}

    # Read current position
    state = ctrl.get_state()
    start_angle = state["angle_rad"]
    target_angle = ctrl._pct_to_rad(target_pct)

    print(f"  Step: {start_angle:.3f} -> {target_angle:.3f} rad "
          f"(Kp={ctrl.kp_track:.1f}, Kd={ctrl.kd_track:.1f})")

    # Start the move
    ctrl.move_to_pct(target_pct)

    t0 = time.monotonic()
    while time.monotonic() - t0 < duration_s:
        fb = ctrl._dev.get_feedback(ctrl._mst_id)
        if fb:
            t = time.monotonic() - t0
            data["t"].append(t)
            data["pos"].append(fb["q"])
            data["vel"].append(fb["dq"])
            data["tau"].append(fb["tau"])
            data["target"].append(target_angle)
            data["err"].append(fb["err"])
        time.sleep(0.005)  # 200Hz logging

    return data


def analyze_response(data: dict, target_angle: float) -> dict:
    """Analyze step response for oscillation metrics."""
    if len(data["t"]) < 20:
        return {"error": "insufficient data"}

    pos = data["pos"]
    t = data["t"]

    # Final position (average of last 20 samples)
    final_pos = sum(pos[-20:]) / 20
    pos_error = abs(final_pos - target_angle)

    # Overshoot: max deviation past target
    if target_angle > pos[0]:  # moving positive
        overshoot = max(pos) - target_angle
    else:
        overshoot = target_angle - min(pos)
    overshoot_pct = abs(overshoot) / abs(target_angle - pos[0]) * 100 if abs(target_angle - pos[0]) > 0.01 else 0

    # Settling time: when pos stays within 2% of final value
    band = abs(target_angle - pos[0]) * 0.02
    if band < 0.005:
        band = 0.005
    settling_time = t[-1]
    for i in range(len(pos) - 1, -1, -1):
        if abs(pos[i] - final_pos) > band:
            settling_time = t[i] if i < len(t) else t[-1]
            break

    # Oscillation detection: count zero-crossings of (pos - target)
    error_signal = [p - target_angle for p in pos]
    # Only look at the last 60% of data (after initial move)
    start_idx = len(error_signal) * 4 // 10
    tail = error_signal[start_idx:]
    tail_t = t[start_idx:]

    zero_crossings = 0
    for i in range(1, len(tail)):
        if tail[i-1] * tail[i] < 0:
            zero_crossings += 1

    # Oscillation amplitude (RMS of tail error)
    tail_rms = math.sqrt(sum(e**2 for e in tail) / len(tail)) if tail else 0

    # Peak-to-peak in tail
    if tail:
        p2p = max(tail) - min(tail)
    else:
        p2p = 0

    # Oscillation frequency estimate
    tail_duration = tail_t[-1] - tail_t[0] if len(tail_t) > 1 else 1
    osc_freq = zero_crossings / (2 * tail_duration) if tail_duration > 0 else 0

    is_oscillating = (zero_crossings >= 4 and p2p > 0.02) or p2p > 0.05

    return {
        "overshoot_pct": round(overshoot_pct, 1),
        "settling_time": round(settling_time, 3),
        "steady_state_error": round(pos_error, 4),
        "zero_crossings": zero_crossings,
        "tail_rms": round(tail_rms, 4),
        "peak_to_peak": round(p2p, 4),
        "osc_freq_hz": round(osc_freq, 1),
        "is_oscillating": is_oscillating,
        "final_pos": round(final_pos, 4),
    }


def print_ascii_plot(data: dict, width: int = 80, height: int = 20):
    """Print ASCII plot of position vs time."""
    pos = data["pos"]
    t = data["t"]
    target = data["target"][0] if data["target"] else 0

    if not pos:
        return

    p_min = min(min(pos), target) - 0.1
    p_max = max(max(pos), target) + 0.1
    t_max = max(t)

    print(f"\n  Position vs Time (target={target:.3f} rad)")
    print(f"  {'─' * width}")

    grid = [[' '] * width for _ in range(height)]

    # Plot target line
    target_row = int((target - p_min) / (p_max - p_min) * (height - 1))
    target_row = max(0, min(height - 1, height - 1 - target_row))
    for col in range(width):
        grid[target_row][col] = '─'

    # Plot position
    for i in range(len(pos)):
        col = int(t[i] / t_max * (width - 1))
        row = int((pos[i] - p_min) / (p_max - p_min) * (height - 1))
        row = max(0, min(height - 1, height - 1 - row))
        col = max(0, min(width - 1, col))
        grid[row][col] = '●'

    for r, row in enumerate(grid):
        if r == 0:
            label = f"{p_max:+.2f}"
        elif r == height - 1:
            label = f"{p_min:+.2f}"
        elif r == target_row:
            label = f"{target:+.2f}"
        else:
            label = "      "
        print(f"  {label:>6s}│{''.join(row)}│")

    print(f"  {'':>6s}└{'─' * width}┘")
    print(f"  {'':>6s} 0{'':>{width//2-1}}t={t_max:.1f}s")


def auto_tune(ctrl: GripperController, test_pct_a: float = 30.0,
              test_pct_b: float = 70.0, max_iterations: int = 8):
    """Iteratively tune Kp/Kd by analyzing step responses."""
    print("\n=== Auto-Tuning ===\n")

    # Start from conservative values
    kp = 15.0
    kd = 2.0
    best_kp = kp
    best_kd = kd
    best_score = float('inf')

    for iteration in range(max_iterations):
        ctrl.kp_track = kp
        ctrl.kd_track = kd
        print(f"\n--- Iteration {iteration + 1}/{max_iterations}: Kp={kp:.1f}, Kd={kd:.1f} ---")

        # Move to position A first
        ctrl.move_to_pct(test_pct_a)
        time.sleep(2.0)

        # Step to position B and record
        data = collect_step_response(ctrl, test_pct_b, duration_s=3.0)
        metrics = analyze_response(data, ctrl._pct_to_rad(test_pct_b))

        print_ascii_plot(data)
        print(f"\n  Metrics: overshoot={metrics['overshoot_pct']:.1f}%, "
              f"settling={metrics['settling_time']:.2f}s, "
              f"p2p={metrics['peak_to_peak']:.4f} rad, "
              f"osc_freq={metrics['osc_freq_hz']:.1f} Hz, "
              f"oscillating={'YES' if metrics['is_oscillating'] else 'no'}")

        # Score: lower is better
        # Penalize oscillation heavily, then settling time
        osc_penalty = metrics['peak_to_peak'] * 100
        settle_penalty = metrics['settling_time']
        overshoot_penalty = metrics['overshoot_pct'] * 0.1
        score = osc_penalty + settle_penalty + overshoot_penalty

        print(f"  Score: {score:.2f} (osc={osc_penalty:.1f} settle={settle_penalty:.1f} "
              f"overshoot={overshoot_penalty:.1f})")

        if score < best_score:
            best_score = score
            best_kp = kp
            best_kd = kd
            print(f"  ★ New best: Kp={best_kp:.1f}, Kd={best_kd:.1f}, score={best_score:.2f}")

        # Adjust parameters for next iteration
        if metrics['is_oscillating']:
            # Oscillating: reduce Kp, increase Kd
            kp *= 0.6
            kd *= 1.3
            print(f"  → Reducing Kp, increasing Kd (oscillation)")
        elif metrics['overshoot_pct'] > 10:
            # Overshoot: increase Kd
            kd *= 1.2
            print(f"  → Increasing Kd (overshoot)")
        elif metrics['settling_time'] > 1.5:
            # Too slow: increase Kp slightly
            kp *= 1.2
            print(f"  → Increasing Kp (slow settling)")
        else:
            print(f"  → Response looks good, stopping early.")
            best_kp = kp
            best_kd = kd
            break

        # Clamp to safe ranges
        kp = max(1.0, min(50.0, kp))
        kd = max(0.5, min(5.0, kd))

    # Apply best parameters
    ctrl.kp_track = best_kp
    ctrl.kd_track = best_kd
    print(f"\n=== Auto-Tune Complete ===")
    print(f"  Best: Kp={best_kp:.1f}, Kd={best_kd:.1f}, score={best_score:.2f}")

    # Final verification
    print(f"\n--- Final Verification ---")
    ctrl.move_to_pct(test_pct_a)
    time.sleep(2.0)
    data = collect_step_response(ctrl, test_pct_b, duration_s=3.0)
    metrics = analyze_response(data, ctrl._pct_to_rad(test_pct_b))
    print_ascii_plot(data)
    print(f"  Final: overshoot={metrics['overshoot_pct']:.1f}%, "
          f"settling={metrics['settling_time']:.2f}s, "
          f"p2p={metrics['peak_to_peak']:.4f}, "
          f"oscillating={'YES' if metrics['is_oscillating'] else 'no'}")

    return best_kp, best_kd


def main():
    dev = DmDevice()
    ctrl = None

    try:
        print("[INIT] Opening CAN-FD ...")
        dev.open(nom_baud_hz=1_000_000, dat_baud_hz=5_000_000, sn=ADAPTER_SN)

        ctrl = GripperController(dev, CAN_ID, MST_ID)

        if os.path.exists(CALIB_PATH):
            ctrl.load_calibration(CALIB_PATH)
        else:
            print("[ERROR] No calibration file. Run gripper_demo.py first to calibrate.")
            return

        print("[INIT] Enabling motor ...")
        ctrl.enable()
        time.sleep(0.5)

        best_kp, best_kd = auto_tune(ctrl)

        save = input("\nSave tuned parameters? (y/N): ").strip().lower()
        if save == "y":
            params = {"kp_track": best_kp, "kd_track": best_kd}
            path = os.path.join(os.path.dirname(__file__), "..", "gripper_tuning.json")
            with open(path, "w") as f:
                json.dump(params, f, indent=2)
            print(f"Saved to {path}")

    except Exception as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        raise
    finally:
        print("[SHUTDOWN] ...")
        if ctrl:
            try:
                ctrl.disable()
            except Exception:
                pass
        dev.close()
        print("[SHUTDOWN] Done.")


if __name__ == "__main__":
    main()
