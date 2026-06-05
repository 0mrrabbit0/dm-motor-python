"""
Probe motor at different baud rate combinations to find the current setting.

Tries all common CAN/CAN-FD baud rate combinations, sends enable + MIT
commands at each, and checks if the motor responds.

Usage:
    LD_LIBRARY_PATH=... PYTHONPATH=src python3 tools/probe_baudrate.py
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "vendor",
                                "dm_device_sdk", "linux", "x86_64"))

ADAPTER_SN = "14AA044B241402B10DDBDAFE448040BB"
CAN_ID = 0x01
MST_ID = 0x11

# All possible baud rate combinations
# CAN_BR register values: 0=125K, 1=250K, 2=500K, 3=1M,
#   4=1M(classic), 5=2M_1M, 6=3M_1M, 7=4M_1M, 8=5M_1M, 9=5M_1M(FD)
BAUD_COMBOS = [
    # (nom_baud, dat_baud, canfd_mode, label)
    (1000000, 1000000, "classic 1M"),
    (500000,  500000,  "classic 500K"),
    (250000,  250000,  "classic 250K"),
    (1000000, 2000000, "FD 1M/2M"),
    (1000000, 3000000, "FD 1M/3M"),
    (1000000, 4000000, "FD 1M/4M"),
    (1000000, 5000000, "FD 1M/5M"),
]


def try_baud(nom, dat, label):
    """Try communicating at a specific baud rate. Returns True if motor responds."""
    from usb_class import usb_class as UsbClass
    from dm_motor.sdk import decode_feedback

    hw = None
    try:
        hw = UsbClass(nom, dat, ADAPTER_SN)
        time.sleep(0.5)

        # Set up feedback collection
        responses = []
        def on_rx(frame):
            try:
                cid = frame.head.id
                if cid != 0x7FF:
                    responses.append(cid)
            except Exception:
                pass

        hw.setFrameCallback(on_rx)
        time.sleep(0.2)

        # Send enable commands
        for _ in range(10):
            hw.fdcanFrameSend([0xff]*7 + [0xfc], CAN_ID)
            time.sleep(0.01)

        # Send MIT commands
        from dm_motor.sdk import encode_mit
        for _ in range(50):
            hw.fdcanFrameSend(list(encode_mit(0, 1, 0, 0, 0)), CAN_ID)
            time.sleep(0.01)

        # Check if we got any response
        got_response = len(responses) > 0

        # Disable
        for _ in range(5):
            hw.fdcanFrameSend([0xff]*7 + [0xfd], CAN_ID)
            time.sleep(0.005)

        return got_response, len(responses)

    except Exception as e:
        return False, str(e)
    finally:
        if hw:
            try:
                hw.close()
            except Exception:
                pass
        time.sleep(0.5)


def main():
    print("=== Baud Rate Probe ===")
    print(f"Adapter SN: {ADAPTER_SN}")
    print(f"Motor CAN_ID: 0x{CAN_ID:02X}, MST_ID: 0x{MST_ID:02X}\n")

    print(f"{'Baud Rate':>15s} | {'Response':>10s} | {'Frames':>8s}")
    print("-" * 45)

    found = None
    for nom, dat, label in BAUD_COMBOS:
        ok, count = try_baud(nom, dat, label)
        status = "YES!" if ok else "no"
        print(f"{label:>15s} | {status:>10s} | {count!s:>8s}")
        if ok and found is None:
            found = (nom, dat, label)

    print()
    if found:
        nom, dat, label = found
        print(f"=== FOUND: Motor responds at {label} ({nom}/{dat}) ===")
        print(f"\nUpdate your code to use:")
        print(f"  dev.open(nom_baud_hz={nom}, dat_baud_hz={dat}, ...)")
    else:
        print("=== Motor did not respond at any baud rate ===")
        print("Possible causes:")
        print("  1. Motor not powered (check 24V and LED)")
        print("  2. CAN wiring disconnected (check H/L/GND)")
        print("  3. Motor CAN_ID changed (not 0x01)")
        print("  4. Adapter hardware fault")


if __name__ == "__main__":
    main()
