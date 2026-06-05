"""
Python wrapper for DaMiao motor control via USB-CANFD adapter.

Uses the usb_class Cython driver (vendor/dm_device_sdk/) for USB-CAN communication.
Supports MIT, POS_VEL, VEL, and POS_FORCE control modes.
"""
from enum import IntEnum
import math
import os
import platform
import struct
import sys
import threading
import time


# ---------- Locate and import usb_class driver ----------

def _find_usb_class_dir() -> str:
    pkg_dir = os.path.dirname(os.path.abspath(__file__))
    proj_root = os.path.dirname(os.path.dirname(pkg_dir))
    machine = platform.machine()
    arch_map = {"x86_64": "x86_64", "aarch64": "arm64", "arm64": "arm64"}
    arch = arch_map.get(machine, machine)
    return os.path.join(proj_root, "vendor", "dm_device_sdk", "linux", arch)

_usb_class_dir = _find_usb_class_dir()
if _usb_class_dir not in sys.path:
    sys.path.insert(0, _usb_class_dir)

try:
    from usb_class import usb_class as _UsbClass, can_value_type
except ImportError as e:
    raise ImportError(
        f"usb_class driver not found in {_usb_class_dir}. "
        f"Ensure usb_class.cpython-*.so is present. Error: {e}"
    ) from e


# ---------- Motor types & limits ----------

class MotorType(IntEnum):
    DM3507 = 0
    DM4310 = 1
    DM4340 = 2
    DM6006 = 3
    DM8006 = 4
    DM8009 = 5
    DM10010L = 6
    DM10010 = 7
    DMH3510 = 8
    DMH6215 = 9
    DM6248 = 10
    DMS3519 = 11
    DMG6220 = 12

# Per-type (PMAX rad, VMAX rad/s, TMAX Nm)
MOTOR_LIMITS = {
    MotorType.DM3507:   (12.5,  30.0,  1.2),
    MotorType.DM4310:   (12.5,  30.0, 10.0),
    MotorType.DM4340:   (12.5,  10.0, 28.0),
    MotorType.DM6006:   (12.5,  45.0, 12.0),
    MotorType.DM8006:   (12.5,  45.0, 20.0),
    MotorType.DM8009:   (12.5,  45.0, 54.0),
    MotorType.DM10010L: (12.5,  25.0, 20.0),
    MotorType.DM10010:  (12.5,  25.0, 50.0),
    MotorType.DMH3510:  (12.5, 200.0,  1.2),
    MotorType.DMH6215:  (12.5, 200.0,  6.0),
    MotorType.DM6248:   (12.5,  45.0, 48.0),
    MotorType.DMS3519:  ( 6.0, 200.0,  3.0),
    MotorType.DMG6220:  (12.5,  18.0, 10.0),
}

PMAX = 12.5
VMAX = 30.0
TMAX = 10.0


class CtrlMode(IntEnum):
    MIT = 1
    POS_VEL = 2
    VEL = 3
    POS_FORCE = 4


# ---------- Encode / decode helpers ----------

def _f2u(x: float, xmin: float, xmax: float, bits: int) -> int:
    span = xmax - xmin
    return max(0, min((1 << bits) - 1, int((x - xmin) / span * ((1 << bits) - 1))))


def _u2f(u: int, xmin: float, xmax: float, bits: int) -> float:
    return (u / ((1 << bits) - 1)) * (xmax - xmin) + xmin


def encode_mit(kp: float, kd: float, q: float, dq: float, tau: float,
               pmax: float = PMAX, vmax: float = VMAX, tmax: float = TMAX) -> bytes:
    kp_u = _f2u(kp, 0, 500, 12)
    kd_u = _f2u(kd, 0, 5, 12)
    q_u = _f2u(q, -pmax, pmax, 16)
    dq_u = _f2u(dq, -vmax, vmax, 12)
    tau_u = _f2u(tau, -tmax, tmax, 12)
    return bytes([
        (q_u >> 8) & 0xff,
        q_u & 0xff,
        dq_u >> 4,
        ((dq_u & 0xf) << 4) | ((kp_u >> 8) & 0xf),
        kp_u & 0xff,
        kd_u >> 4,
        ((kd_u & 0xf) << 4) | ((tau_u >> 8) & 0xf),
        tau_u & 0xff,
    ])


def decode_feedback(payload, pmax: float = PMAX, vmax: float = VMAX,
                    tmax: float = TMAX):
    if hasattr(payload, 'data'):
        d = payload.data
    else:
        d = payload
    err = (d[0] >> 4) & 0xf
    q_u = (d[1] << 8) | d[2]
    dq_u = (d[3] << 4) | (d[4] >> 4)
    tau_u = ((d[4] & 0xf) << 8) | d[5]
    q = _u2f(q_u, -pmax, pmax, 16)
    dq = _u2f(dq_u, -vmax, vmax, 12)
    tau = _u2f(tau_u, -tmax, tmax, 12)
    t_mos = d[6]
    t_rotor = d[7]
    return dict(err=err, q=q, dq=dq, tau=tau, t_mos=t_mos, t_rotor=t_rotor)


def encode_pos_force(p_des: float, v_des: float, i_des: float) -> bytes:
    """Encode POS_FORCE frame: position + velocity limit + current limit."""
    v_u16 = max(0, min(0xFFFF, int(v_des * 100)))
    i_u16 = max(0, min(10000, int(i_des * 10000)))
    return struct.pack('<f', p_des) + struct.pack('<HH', v_u16, i_u16)


def encode_pos_vel(pos: float, vel: float) -> bytes:
    """Encode POS_VEL frame: position + velocity setpoint."""
    if math.isnan(pos) or math.isinf(pos) or math.isnan(vel) or math.isinf(vel):
        raise ValueError(f"pos_vel params must be finite: pos={pos}, vel={vel}")
    return struct.pack('<ff', pos, vel)


# ---------- High-level wrapper ----------

# Kept for backward compat — unused by the new usb_class backend
REC_CALLBACK = None


class DmDevice:
    """Motor controller wrapping the usb_class Cython driver."""

    MIT_OFFSET = 0x000
    POS_VEL_OFFSET = 0x100
    VEL_OFFSET = 0x200
    POS_FORCE_OFFSET = 0x300

    def __init__(self, motor_type: MotorType = MotorType.DM4310):
        self.motor_type = motor_type
        self.pmax, self.vmax, self.tmax = MOTOR_LIMITS[motor_type]
        self._latest = {}
        self._lock = threading.Lock()
        self._hw = None
        self.sent_count = 0
        self.err_count = 0

    def _on_rx(self, frame):
        """Callback from usb_class when a CAN frame is received."""
        cid = frame.head.id
        if cid == 0x7FF:
            return
        try:
            fb = decode_feedback(frame, self.pmax, self.vmax, self.tmax)
        except Exception:
            return
        with self._lock:
            self._latest[cid] = fb

    def open(self, nom_baud_hz: int = 1_000_000, dat_baud_hz: int = 5_000_000,
             sn: str = None, **_kwargs):
        """Open the USB-CANFD adapter.

        Args:
            nom_baud_hz: arbitration baud rate (default 1M)
            dat_baud_hz: data baud rate for CAN-FD (default 5M)
            sn: adapter serial number (auto-detect if None)
        """
        if sn is None:
            from usb_class import dm_device
            dev = dm_device()
            found = dev.usb_get_dm_device()
            if not found:
                raise RuntimeError("no USB-CANFD device found")
            sn = found[0] if isinstance(found, (list, tuple)) else found

        self._sn = sn
        self._nom_baud = nom_baud_hz
        self._dat_baud = dat_baud_hz
        self._hw = _UsbClass(nom_baud_hz, dat_baud_hz, sn)
        time.sleep(0.5)
        self._hw.setFrameCallback(lambda val: self._on_rx(val))
        time.sleep(0.2)
        print(f"[DM] opened, SN={sn}, baud={nom_baud_hz//1000}K/{dat_baud_hz//1000}K",
              file=sys.stderr)

    def close(self):
        if self._hw is not None:
            try:
                self._hw.close()
            except Exception:
                pass
            self._hw = None

    def reset(self, pause_s: float = 4.0):
        """Reset the CAN bus by closing and reopening the USB adapter.

        The pause lets the motor's comm timeout fire, which transitions
        it from error → disabled. After reopening, explicit disable
        commands clear any remaining error latch.
        """
        sn = self._sn
        nom = self._nom_baud
        dat = self._dat_baud
        print(f"[DM] Resetting CAN bus (pause {pause_s}s)...", file=sys.stderr)
        self.close()
        time.sleep(pause_s)
        self.open(nom_baud_hz=nom, dat_baud_hz=dat, sn=sn)
        # Clear error latch → disable → enable sequence
        for can_id in [0x01, 0x02, 0x03]:
            for _ in range(5):
                self.clear_error(can_id)
                time.sleep(0.005)
            for _ in range(5):
                self.disable(can_id)
                time.sleep(0.005)
        time.sleep(0.5)
        print("[DM] CAN bus reset complete.", file=sys.stderr)

    def send(self, can_id: int, payload, **_kwargs):
        """Send a CAN frame."""
        if isinstance(payload, bytes):
            payload = list(payload)
        self._hw.fdcanFrameSend(payload, can_id)

    # --- DM motor enable/disable/clear ---
    def enable(self, can_id: int, mode_offset: int = MIT_OFFSET):
        self.send(can_id + mode_offset, [0xff]*7 + [0xfc])

    def disable(self, can_id: int, mode_offset: int = MIT_OFFSET):
        self.send(can_id + mode_offset, [0xff]*7 + [0xfd])

    def clear_error(self, can_id: int, mode_offset: int = MIT_OFFSET):
        """Send clear-error command (0xFB). Clears latched fault states."""
        self.send(can_id + mode_offset, [0xff]*7 + [0xfb])

    # --- MIT mode (CTRL_MODE=1) ---
    def control_mit(self, can_id: int, kp: float, kd: float,
                    q: float, dq: float, tau: float):
        self.send(can_id + self.MIT_OFFSET,
                  list(encode_mit(kp, kd, q, dq, tau,
                                  self.pmax, self.vmax, self.tmax)))

    # --- POS_VEL mode (CTRL_MODE=2) ---
    def control_pos_vel(self, can_id: int, pos: float, vel: float):
        self.send(can_id + self.POS_VEL_OFFSET,
                  list(encode_pos_vel(pos, vel)))

    # --- VEL mode (CTRL_MODE=3) ---
    def control_vel(self, can_id: int, vel_radps: float):
        self.send(can_id + self.VEL_OFFSET,
                  list(struct.pack('<f', vel_radps)))

    # --- POS_FORCE mode (CTRL_MODE=4) ---
    def control_pos_force(self, can_id: int, p_des: float,
                          v_des: float, i_des: float):
        self.send(can_id + self.POS_FORCE_OFFSET,
                  list(encode_pos_force(p_des, v_des, i_des)))

    # --- Feedback ---
    def get_feedback(self, mst_id: int):
        with self._lock:
            return self._latest.get(mst_id)

    # --- Zero position ---
    def set_zero(self, can_id: int, mode_offset: int = MIT_OFFSET):
        self.send(can_id + mode_offset, [0xff] * 7 + [0xfe])

    # --- Motor parameter read/write/save ---
    def write_param(self, can_id: int, rid: int, value_u32: int):
        idl = can_id & 0xff
        idh = (can_id >> 8) & 0xff
        v = value_u32 & 0xffffffff
        self.send(0x7FF, [
            idl, idh, 0x55, rid,
            v & 0xff, (v >> 8) & 0xff, (v >> 16) & 0xff, (v >> 24) & 0xff,
        ])

    def read_param(self, can_id: int, rid: int):
        idl = can_id & 0xff
        idh = (can_id >> 8) & 0xff
        self.send(0x7FF, [idl, idh, 0x33, rid, 0, 0, 0, 0])

    def save_params(self, can_id: int):
        idl = can_id & 0xff
        idh = (can_id >> 8) & 0xff
        self.send(0x7FF, [idl, idh, 0xAA, 0x01, 0, 0, 0, 0])

    def switch_ctrl_mode(self, can_id: int, mode: CtrlMode):
        self.write_param(can_id, 10, int(mode))
