"""
Smoke test: enable J4310 (canID=1, MASTER_ID=0x11) in MIT mode,
hold position with light damping, print feedback, then disable.

Adjust kp/kd/q_des below to change behavior. Defaults are very gentle.
"""
import sys
import time
import signal

from dm_sdk import DmDevice

CAN_ID = 0x01
MST_ID = 0x11

# Gentle motion: hold current position with light damping, no extra torque
KP = 0.0       # position gain (0 = no position hold)
KD = 1.0       # velocity damping (motor will resist motion)
Q_DES = 0.0    # position setpoint (rad)
DQ_DES = 0.0   # velocity setpoint (rad/s)
TAU = 0.0      # feed-forward torque (Nm)

DURATION_S = 3.0
PERIOD_S = 0.005   # 200Hz control loop


running = True


def on_sigint(signum, frame):
    global running
    running = False


def main():
    signal.signal(signal.SIGINT, on_sigint)
    dev = DmDevice()
    try:
        dev.open(nom_baud_hz=1_000_000, canfd=False)
        time.sleep(0.2)

        print("[TEST] enabling motor", file=sys.stderr)
        for _ in range(5):
            dev.enable(CAN_ID)
            time.sleep(0.005)
        time.sleep(0.1)

        fb = dev.get_feedback(MST_ID)
        print(f"[TEST] post-enable feedback: {fb}", file=sys.stderr)

        t0 = time.monotonic()
        next_log = 0.0
        while running and (time.monotonic() - t0) < DURATION_S:
            dev.control_mit(CAN_ID, KP, KD, Q_DES, DQ_DES, TAU)
            t = time.monotonic() - t0
            if t >= next_log:
                fb = dev.get_feedback(MST_ID)
                print(f"[TEST] t={t:.2f}s fb={fb}", file=sys.stderr)
                next_log = t + 0.5
            time.sleep(PERIOD_S)

        print("[TEST] disabling motor", file=sys.stderr)
        for _ in range(5):
            dev.disable(CAN_ID)
            time.sleep(0.005)
        time.sleep(0.05)
    finally:
        dev.close()


if __name__ == "__main__":
    main()
