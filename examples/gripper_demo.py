"""
Interactive CLI demo for the DM4310 force-position hybrid gripper.

Motor → lead screw → sleeve → linkage → jaws.
First run requires calibration to map motor angles to jaw opening.
"""
import os
import sys
import time

from dm_motor import DmDevice, GripperController, GripperMode, GripperState

CAN_ID = 0x01
MST_ID = 0x11
ADAPTER_SN = "14AA044B241402B10DDBDAFE448040BB"

CALIB_PATH = os.path.join(os.path.dirname(__file__), "..", "gripper_calibration.json")


def print_state(ctrl: GripperController):
    s = ctrl.get_state()
    pct_str = f"{s['opening_pct']:5.1f}%" if s['opening_pct'] is not None else "  N/A"
    mm_str = f"{s['opening_mm']:5.1f}mm" if s['opening_mm'] is not None else "  N/A"
    print(f"  {s['state']:10s}  angle={s['angle_rad']:+7.3f} rad  "
          f"open={pct_str} {mm_str}  "
          f"tau={s['torque']:+5.2f} Nm  err={s['error']}  "
          f"loop={'OK' if s['loop_alive'] else 'DEAD'}")


def on_state_change(old: GripperState, new: GripperState):
    print(f"  [state] {old.value} -> {new.value}")


def main():
    dev = DmDevice()
    ctrl = None

    try:
        print("[INIT] Opening CAN-FD 1M/5M ...")
        dev.open(nom_baud_hz=1_000_000, dat_baud_hz=5_000_000, sn=ADAPTER_SN)

        # Quick motor check — reset CAN if in error state
        for _ in range(5):
            dev.enable(CAN_ID)
            time.sleep(0.005)
        time.sleep(0.2)
        for _ in range(20):
            dev.control_mit(CAN_ID, kp=0, kd=1.0, q=0, dq=0, tau=0)
            time.sleep(0.005)
        fb = dev.get_feedback(MST_ID)
        if not fb or fb["err"] not in (0, 1):
            print("[RECOVER] Motor error, resetting CAN bus...")
            dev.reset()
        for _ in range(3):
            dev.disable(CAN_ID)
            time.sleep(0.005)
        time.sleep(0.1)

        ctrl = GripperController(
            dev, CAN_ID, MST_ID,
            on_state_change=on_state_change,
        )

        # Load calibration (zero point persisted in motor flash via save_params)
        if os.path.exists(CALIB_PATH):
            ctrl.load_calibration(CALIB_PATH)
        else:
            print("\n[CALIB] First-time setup. Running full calibration...")
            ctrl.calibrate_auto()
            ctrl.save_calibration(CALIB_PATH)

        # Load LuGre friction model if available
        ctrl.load_friction()

        print("[INIT] Enabling motor ...")
        ctrl.enable()
        time.sleep(0.1)

        print()
        print("=== DM4310 Gripper Demo ===")
        print()

        while True:
            print("[1] Move to angle (rad)")
            print("[2] Move to opening (%%)")
            if ctrl._stroke_mm:
                print("[3] Move to opening (mm)")
            print("[4] Torque mode")
            print("[5] Hybrid: close with force limit")
            print("[6] Open gripper")
            print("[7] Close gripper")
            print("[8] Monitor state")
            print("[9] Re-calibrate")
            print("[0] Exit")
            print()

            try:
                choice = input(">>> ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if choice == "1":
                try:
                    angle = float(input("  angle (rad): "))
                except ValueError:
                    print("  invalid"); continue
                ctrl.move_to_angle(angle)
                print(f"  -> moving to {angle:.3f} rad")

            elif choice == "2":
                try:
                    pct = float(input("  opening %% (0=closed, 100=open): "))
                except ValueError:
                    print("  invalid"); continue
                ctrl.move_to_pct(pct)
                print(f"  -> moving to {pct:.0f}%%")

            elif choice == "3" and ctrl._stroke_mm:
                try:
                    mm = float(input("  opening (mm): "))
                except ValueError:
                    print("  invalid"); continue
                ctrl.move_to_mm(mm)
                print(f"  -> moving to {mm:.1f} mm")

            elif choice == "4":
                try:
                    tau = float(input("  torque (Nm): "))
                except ValueError:
                    print("  invalid"); continue
                ctrl.set_torque(tau)
                print(f"  -> torque mode: {tau:.2f} Nm")

            elif choice == "5":
                try:
                    pct = float(input("  target opening %% (0=closed): "))
                    force = float(input("  force limit (Nm): "))
                except ValueError:
                    print("  invalid"); continue
                ctrl.grip_pct(pct, force)
                print(f"  -> hybrid: {pct:.0f}%%, limit={force:.2f} Nm")

            elif choice == "6":
                use_force = input("  force limit? (Enter=none, or Nm value): ").strip()
                if use_force:
                    try:
                        ctrl.open_gripper(force_limit=float(use_force))
                    except ValueError:
                        print("  invalid"); continue
                else:
                    ctrl.open_gripper()
                print("  -> opening")

            elif choice == "7":
                use_force = input("  force limit? (Enter=none, or Nm value): ").strip()
                if use_force:
                    try:
                        ctrl.close_gripper(force_limit=float(use_force))
                    except ValueError:
                        print("  invalid"); continue
                else:
                    ctrl.close_gripper()
                print("  -> closing")

            elif choice == "8":
                try:
                    dur = float(input("  duration (s, default 3): ").strip() or "3")
                except ValueError:
                    dur = 3.0
                t0 = time.monotonic()
                while time.monotonic() - t0 < dur:
                    print_state(ctrl)
                    time.sleep(0.1)

            elif choice == "9":
                ctrl.disable()
                ctrl.calibrate_auto()
                ctrl.save_calibration(CALIB_PATH)
                ctrl.enable()

            elif choice == "0":
                break
            else:
                print("  unknown option")
            print()

    except Exception as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        raise
    finally:
        print("[SHUTDOWN] Disabling ...")
        if ctrl is not None:
            try:
                ctrl.disable()
            except Exception:
                pass
        dev.close()
        print("[SHUTDOWN] Done.")


if __name__ == "__main__":
    main()
