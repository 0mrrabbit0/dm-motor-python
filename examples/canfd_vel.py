"""
Test motor at 5Mbps CAN-FD in VEL (speed) mode.
Motor is currently in CTRL_MODE=3 (speed mode).
"""
import sys
import time
import signal

from dm_motor import DmDevice

CAN_ID = 0x01
MST_ID = 0x11

VEL_TARGET = 2.0     # rad/s — gentle constant speed

DURATION_S = 3.0
PERIOD_S = 0.005     # 200Hz


running = True


def on_sigint(signum, frame):
    global running
    running = False


def main():
    signal.signal(signal.SIGINT, on_sigint)
    dev = DmDevice()
    try:
        # CAN-FD: 1M arbitration, 5M data, BRS on
        dev.open(nom_baud_hz=1_000_000, dat_baud_hz=5_000_000, canfd=True)
        time.sleep(0.2)

        print(f"[TEST5M] enabling motor (VEL mode, ID 0x{CAN_ID + dev.VEL_OFFSET:X})",
              file=sys.stderr)
        for _ in range(5):
            dev.enable(CAN_ID, mode_offset=dev.VEL_OFFSET)
            time.sleep(0.005)
        time.sleep(0.1)

        fb = dev.get_feedback(MST_ID)
        print(f"[TEST5M] post-enable feedback: {fb}", file=sys.stderr)

        print(f"[TEST5M] driving at {VEL_TARGET} rad/s for {DURATION_S}s",
              file=sys.stderr)
        t0 = time.monotonic()
        next_log = 0.0
        while running and (time.monotonic() - t0) < DURATION_S:
            dev.control_vel(CAN_ID, VEL_TARGET)
            t = time.monotonic() - t0
            if t >= next_log:
                fb = dev.get_feedback(MST_ID)
                print(f"[TEST5M] t={t:.2f}s fb={fb}", file=sys.stderr)
                next_log = t + 0.5
            time.sleep(PERIOD_S)

        # ramp down to 0 before disabling for soft stop
        print("[TEST5M] ramping to 0", file=sys.stderr)
        for _ in range(50):
            dev.control_vel(CAN_ID, 0.0)
            time.sleep(0.01)

        print("[TEST5M] disabling motor", file=sys.stderr)
        for _ in range(5):
            dev.disable(CAN_ID, mode_offset=dev.VEL_OFFSET)
            time.sleep(0.005)
        time.sleep(0.05)
    finally:
        dev.close()


if __name__ == "__main__":
    main()
