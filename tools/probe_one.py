"""
Single-combo 5M FD probe. Run via subprocess to keep SDK state clean.
Args: <brs:0|1> <nom_sp> <dat_sp>
Prints summary line: "RESULT brs=X nom_sp=Y dat_sp=Z rx_count=N"
"""
import sys
import time
import threading

from dm_motor import DmDevice, REC_CALLBACK


def main():
    if len(sys.argv) < 4:
        print("usage: probe_one.py <brs:0|1> <nom_sp> <dat_sp>", file=sys.stderr)
        sys.exit(2)
    brs = bool(int(sys.argv[1]))
    nom_sp = float(sys.argv[2])
    dat_sp = float(sys.argv[3])

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

        # Enable in VEL mode (motor's CTRL_MODE=3 → ID = 0x201)
        for _ in range(5):
            dev.enable(0x01, mode_offset=dev.VEL_OFFSET)
            time.sleep(0.005)
        time.sleep(0.1)

        # Send vel=0 a few times to elicit feedback
        for _ in range(20):
            dev.control_vel(0x01, 0.0)
            time.sleep(0.01)

        for _ in range(5):
            dev.disable(0x01, mode_offset=dev.VEL_OFFSET)
            time.sleep(0.005)
        time.sleep(0.05)
    except Exception as e:
        print(f"EXCEPTION: {e}", file=sys.stderr)
    finally:
        dev.close()

    with rx_lock:
        n = len(rx)
        print(f"RESULT brs={int(brs)} nom_sp={nom_sp} dat_sp={dat_sp} rx_count={n}")
        for i, (cid, dlc, payload) in enumerate(rx[:3]):
            print(f"  [{i}] id=0x{cid:X} dlc={dlc} data={[hex(b) for b in payload]}")


if __name__ == "__main__":
    main()
