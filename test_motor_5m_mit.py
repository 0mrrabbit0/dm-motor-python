"""
Drive motor in MIT mode at 5M FD.
Motor is at CTRL_MODE=1 (MIT), CAN_BR=9 (5M FD), CAN_ID=0x01, MST_ID=0x11.
"""
import sys
import time
import signal

from dm_sdk import DmDevice

CAN_ID = 0x01
MST_ID = 0x11

# MIT params: kp=0, kd=2 → motor follows desired velocity (no position hold)
KP = 0.0
KD = 2.0
Q_DES = 0.0       # ignored when kp=0
DQ_DES = 2.0      # rad/s
TAU = 0.0

DURATION_S = 3.0
PERIOD_S = 0.005   # 200Hz

running = True


def on_sigint(signum, frame):
    global running
    running = False


def main():
    signal.signal(signal.SIGINT, on_sigint)
    dev = DmDevice()
    try:
        dev.open(nom_baud_hz=1_000_000, dat_baud_hz=5_000_000,
                 canfd=True, brs=True)
        time.sleep(0.2)

        # MIT-mode enable: send to canID + 0x000 = 0x001
        print(f"[TEST] enabling motor (MIT mode, ID 0x{CAN_ID:X})", file=sys.stderr)
        for _ in range(5):
            dev.enable(CAN_ID, mode_offset=dev.MIT_OFFSET)
            time.sleep(0.005)
        time.sleep(0.1)

        fb = dev.get_feedback(MST_ID)
        print(f"[TEST] post-enable feedback: {fb}", file=sys.stderr)

        print(f"[TEST] driving with MIT kp={KP} kd={KD} dq_des={DQ_DES} for {DURATION_S}s",
              file=sys.stderr)
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

        # Ramp to 0 before disable for soft stop
        print("[TEST] ramping to 0", file=sys.stderr)
        for _ in range(50):
            dev.control_mit(CAN_ID, 0.0, KD, 0.0, 0.0, 0.0)
            time.sleep(0.01)

        for _ in range(5):
            dev.disable(CAN_ID, mode_offset=dev.MIT_OFFSET)
            time.sleep(0.005)
        time.sleep(0.05)
    finally:
        dev.close()


if __name__ == "__main__":
    main()
