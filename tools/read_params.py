"""
Read motor's stored params via CAN to confirm CTRL_MODE / CAN_BR / etc.
Operates at 5M FD (motor is currently at 5M).
"""
import sys
import time
import threading

from dm_motor import DmDevice, REC_CALLBACK

CAN_ID = 0x01

PARAMS = [
    (7, "MST_ID"),
    (8, "ESC_ID"),
    (9, "TIMEOUT"),
    (10, "CTRL_MODE"),
    (35, "CAN_BR"),
]


def main():
    rx_lock = threading.Lock()
    seen = []

    dev = DmDevice()

    def cb(frame_ptr):
        f = frame_ptr.contents
        with rx_lock:
            seen.append((f.head.can_id,
                         bytes(f.payload[:f.head.dlc])))
        dev._on_rx(frame_ptr)

    dev._rec_cb = REC_CALLBACK(cb)

    try:
        dev.open(nom_baud_hz=1_000_000, dat_baud_hz=5_000_000,
                 canfd=True, brs=True)
        time.sleep(0.2)

        for rid, name in PARAMS:
            with rx_lock:
                seen.clear()
            dev.read_param(CAN_ID, rid)
            time.sleep(0.15)
            with rx_lock:
                got = None
                for cid, p in seen:
                    if len(p) >= 8 and p[2] == 0x33 and p[3] == rid:
                        v = p[4] | (p[5] << 8) | (p[6] << 16) | (p[7] << 24)
                        got = v
                        break
            print(f"  RID={rid:3d} {name:10s} = {got}")
    finally:
        dev.close()


if __name__ == "__main__":
    main()
