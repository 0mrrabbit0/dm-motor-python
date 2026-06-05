"""
Find the maximum stable Kp for the lead screw gripper.
Sweeps Kp from 1 to 15 in steps, tests each for 2 seconds,
reports the oscillation boundary.
"""
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from dm_motor import DmDevice

ADAPTER_SN = "14AA044B241402B10DDBDAFE448040BB"
CAN_ID = 0x01
MST_ID = 0x11
KD = 2.0
RATE_HZ = 200
TEST_DURATION = 2.0


def test_kp(dev, kp, hold_pos):
    """Test a Kp value for oscillation. Returns (stable, metrics)."""
    period = 1.0 / RATE_HZ
    positions = []

    for _ in range(int(TEST_DURATION * RATE_HZ)):
        dev.control_mit(CAN_ID, kp=kp, kd=KD, q=hold_pos, dq=0, tau=0)
        time.sleep(period)
        fb = dev.get_feedback(MST_ID)
        if fb:
            positions.append(fb["q"])
            if fb["err"] == 0:
                dev.enable(CAN_ID)
                time.sleep(0.005)

    if len(positions) < 50:
        return True, {"error": "no data"}

    # Use last 60% of data (skip transient)
    tail = positions[len(positions) * 4 // 10:]
    mean = sum(tail) / len(tail)
    p2p = max(tail) - min(tail)
    std = math.sqrt(sum((p - mean)**2 for p in tail) / len(tail))

    # Count velocity sign changes (proxy for oscillation)
    vel_changes = 0
    for i in range(2, len(tail)):
        d1 = tail[i-1] - tail[i-2]
        d2 = tail[i] - tail[i-1]
        if d1 * d2 < 0:
            vel_changes += 1

    duration = len(tail) * period
    vel_change_rate = vel_changes / duration if duration > 0 else 0

    is_oscillating = p2p > 0.02 or (std > 0.005 and vel_change_rate > 20)

    return not is_oscillating, {
        "kp": kp,
        "p2p": round(p2p, 5),
        "std": round(std, 5),
        "vel_change_rate": round(vel_change_rate, 1),
        "oscillating": is_oscillating,
    }


def main():
    dev = DmDevice()
    try:
        dev.open(nom_baud_hz=1_000_000, dat_baud_hz=5_000_000, sn=ADAPTER_SN)
        for _ in range(5):
            dev.enable(CAN_ID)
            time.sleep(0.005)
        time.sleep(0.3)

        # Get current position
        for _ in range(50):
            dev.control_mit(CAN_ID, kp=0, kd=2, q=0, dq=0, tau=0)
            time.sleep(0.005)
        fb = dev.get_feedback(MST_ID)
        hold_pos = fb["q"] if fb else 0
        print(f"Hold position: {hold_pos:.4f} rad\n")

        print(f"{'Kp':>5s} | {'p2p':>8s} | {'std':>8s} | {'vel_chg/s':>9s} | Result")
        print("-" * 55)

        max_stable_kp = 0
        kp_values = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 15]

        for kp in kp_values:
            # Brief damping between tests
            for _ in range(50):
                dev.control_mit(CAN_ID, kp=0, kd=2, q=0, dq=0, tau=0)
                time.sleep(0.005)

            stable, m = test_kp(dev, kp, hold_pos)
            status = "  STABLE" if stable else "  !! OSC"
            print(f"{kp:5.0f} | {m.get('p2p',0):8.5f} | {m.get('std',0):8.5f} | "
                  f"{m.get('vel_change_rate',0):9.1f} | {status}")

            if stable:
                max_stable_kp = kp
            elif max_stable_kp > 0:
                # Found the boundary, no need to test higher
                break

        # Damping then disable
        for _ in range(50):
            dev.control_mit(CAN_ID, kp=0, kd=2, q=0, dq=0, tau=0)
            time.sleep(0.01)
        for _ in range(5):
            dev.disable(CAN_ID)
            time.sleep(0.005)

        print(f"\n=== Maximum stable Kp = {max_stable_kp} (with Kd={KD}) ===")
        print(f"Recommended: Kp={max_stable_kp}, Kd={KD}")

        result = {"max_stable_kp": max_stable_kp, "kd": KD, "rate_hz": RATE_HZ}
        path = os.path.join(os.path.dirname(__file__), "..", "diag_data", "stable_kp.json")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump(result, f, indent=2)
        print(f"Saved to {path}")

    finally:
        try:
            for _ in range(5):
                dev.disable(CAN_ID)
                time.sleep(0.005)
        except Exception:
            pass
        dev.close()


if __name__ == "__main__":
    main()
