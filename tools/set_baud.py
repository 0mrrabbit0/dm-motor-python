"""
Force write CAN_BR=9 (= 5Mbps, FD mode) to motor flash via direct CAN protocol.
Bypasses DMTool's UI quirks and writes RID=35 directly via the same write
command (0x55) that worked for MST_ID.

Sequence:
  1. Connect at 1Mbps classic CAN (current motor baud)
  2. Disable motor
  3. Read CAN_BR back to confirm we can talk to it
  4. Write CAN_BR (RID=35) = 9
  5. Save to flash
  6. Motor will auto-restart at 5Mbps; this script will lose comms (expected)

After this script:
  - Power-cycle motor and check serial banner: should show "CAN Baud: 5.00Mbps"
  - If yes: latest firmware accepts 5M. Use test_motor_5m.py to drive it.
  - If no: firmware silently rejected the write. Stuck at 1M.
"""
import sys
import time
import threading
from dm_motor import DmDevice, REC_CALLBACK

CAN_ID = 0x01
RID_CAN_BR = 35
TARGET_BR_CODE = 9  # = 5Mbps per manual baud table


def main():
    dev = DmDevice()

    # Hijack the rx callback BEFORE open(), so the SDK gets wired to ours.
    seen_params = []
    seen_lock = threading.Lock()

    def custom_rx(frame_ptr):
        f = frame_ptr.contents
        cid = f.head.can_id
        dlc = f.head.dlc
        payload = bytes(f.payload[:dlc])
        with seen_lock:
            seen_params.append((cid, payload))
        # also feed feedback decode
        dev._on_rx(frame_ptr)

    dev._rec_cb = REC_CALLBACK(custom_rx)

    try:
        dev.open(nom_baud_hz=1_000_000, canfd=False)
        time.sleep(0.2)

        # 1) disable motor before changing config
        print("[BAUD] disabling motor", file=sys.stderr)
        for _ in range(5):
            dev.disable(CAN_ID)
            time.sleep(0.005)
        time.sleep(0.1)

        # 2) read current CAN_BR — confirms we can talk + parameter protocol works
        print("[BAUD] reading current CAN_BR (RID=35)", file=sys.stderr)
        with seen_lock:
            seen_params.clear()
        dev.read_param(CAN_ID, RID_CAN_BR)
        time.sleep(0.2)
        with seen_lock:
            for cid, p in seen_params:
                if len(p) >= 8 and p[2] == 0x33 and p[3] == RID_CAN_BR:
                    val = p[4] | (p[5] << 8) | (p[6] << 16) | (p[7] << 24)
                    print(f"[BAUD]   current CAN_BR = {val}  (4=1M, 9=5M)",
                          file=sys.stderr)

        # 3) write CAN_BR = 9
        print(f"[BAUD] writing CAN_BR = {TARGET_BR_CODE}", file=sys.stderr)
        with seen_lock:
            seen_params.clear()
        dev.write_param(CAN_ID, RID_CAN_BR, TARGET_BR_CODE)
        time.sleep(0.2)
        with seen_lock:
            ack = False
            for cid, p in seen_params:
                if len(p) >= 8 and p[2] == 0x55 and p[3] == RID_CAN_BR:
                    val = p[4] | (p[5] << 8) | (p[6] << 16) | (p[7] << 24)
                    print(f"[BAUD]   write ACK with stored val = {val}",
                          file=sys.stderr)
                    ack = (val == TARGET_BR_CODE)
            if not ack:
                print("[BAUD]   no write ACK received (motor may have rejected "
                      "the value, or ACK at new baud)", file=sys.stderr)

        # 4) save to flash
        print("[BAUD] sending save command (0xAA)", file=sys.stderr)
        dev.save_params(CAN_ID)
        time.sleep(0.5)
        print("[BAUD] save sent. Motor may auto-restart at new baud.",
              file=sys.stderr)
        print("[BAUD] Done. Check motor's serial boot banner to verify CAN Baud.",
              file=sys.stderr)

    finally:
        dev.close()


if __name__ == "__main__":
    main()
