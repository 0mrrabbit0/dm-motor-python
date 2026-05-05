"""
Probe FD-mode parameters (BRS, sample point) to find what motor accepts.
Sends a no-op MIT-style enable + a few control frames in VEL mode, listens
for any feedback. Whichever combo yields RX frames wins.

Motor must be at 5Mbps FD (CAN_BR=9) and CTRL_MODE=3 (VEL).
"""
import sys
import time
import threading

from dm_sdk import DmDevice, REC_CALLBACK

CAN_ID = 0x01

# (label, brs, nom_sp, dat_sp)
COMBOS = [
    ("BRS=off, nom_sp=0.75, dat_sp=0.75", False, 0.75, 0.75),
    ("BRS=on,  nom_sp=0.75, dat_sp=0.75", True,  0.75, 0.75),
    ("BRS=on,  nom_sp=0.75, dat_sp=0.80", True,  0.75, 0.80),
    ("BRS=on,  nom_sp=0.875,dat_sp=0.875", True, 0.875, 0.875),
    ("BRS=on,  nom_sp=0.80, dat_sp=0.80", True,  0.80, 0.80),
    ("BRS=on,  nom_sp=0.70, dat_sp=0.70", True,  0.70, 0.70),
]


def try_combo(label, brs, nom_sp, dat_sp):
    print(f"\n=== {label} ===", file=sys.stderr)
    rx = []
    rx_lock = threading.Lock()

    dev = DmDevice()

    def cb(frame_ptr):
        f = frame_ptr.contents
        with rx_lock:
            rx.append((f.head.can_id, f.head.dlc,
                       bytes(f.payload[:f.head.dlc])))
        dev._on_rx(frame_ptr)

    dev._rec_cb = REC_CALLBACK(cb)

    try:
        dev.open(nom_baud_hz=1_000_000, dat_baud_hz=5_000_000,
                 canfd=True, brs=brs, nom_sp=nom_sp, dat_sp=dat_sp)
        time.sleep(0.2)

        # Try VEL-mode enable (motor's CTRL_MODE=3, ID = 0x01 + 0x200 = 0x201)
        print("  → enable @ 0x201 (VEL)", file=sys.stderr)
        for _ in range(5):
            dev.enable(CAN_ID, mode_offset=dev.VEL_OFFSET)
            time.sleep(0.005)
        time.sleep(0.1)

        # Send a few VEL=0 commands to elicit feedback
        for _ in range(20):
            dev.control_vel(CAN_ID, 0.0)
            time.sleep(0.01)

        # disable
        for _ in range(5):
            dev.disable(CAN_ID, mode_offset=dev.VEL_OFFSET)
            time.sleep(0.005)
        time.sleep(0.05)
    except Exception as e:
        print(f"  ! exception: {e}", file=sys.stderr)
        return 0
    finally:
        dev.close()

    with rx_lock:
        n = len(rx)
        if n > 0:
            print(f"  ✓ got {n} RX frame(s)", file=sys.stderr)
            for i, (cid, dlc, payload) in enumerate(rx[:3]):
                print(f"    [{i}] id=0x{cid:X} dlc={dlc} "
                      f"data={[hex(b) for b in payload]}",
                      file=sys.stderr)
        else:
            print("  ✗ 0 RX frames", file=sys.stderr)
    return n


def main():
    results = []
    for combo in COMBOS:
        n = try_combo(*combo)
        results.append((combo[0], n))
        time.sleep(0.5)  # small gap between attempts to clear errors

    print("\n=== SUMMARY ===", file=sys.stderr)
    for label, n in results:
        marker = "✓" if n > 0 else "✗"
        print(f"{marker} {label} : {n} RX", file=sys.stderr)


if __name__ == "__main__":
    main()
