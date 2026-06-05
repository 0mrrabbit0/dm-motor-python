"""
DM4310 Gripper SDK — simple API for robotic arm end-effector.

Usage:
    from dm_gripper import Gripper

    with Gripper() as g:
        g.set_position(50)                    # open to 50%
        g.set_position(0, force_limit=0.5)    # close with 0.5Nm limit
        g.set_torque(0.3)                     # constant grip force
        print(g.get_state())
"""
from __future__ import annotations

import json
import os
import time
from typing import Optional

from dm_motor.sdk import DmDevice
from dm_motor.gripper import GripperController, GripperMode

_PKG_DIR = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_CALIB = os.path.join(_PKG_DIR, "calibration.json")
_DEFAULT_FRICTION = os.path.join(_PKG_DIR, "friction.json")


class Gripper:
    """High-level gripper SDK for robotic arm integration.

    Args:
        can_id: Motor CAN ID (default 0x01)
        mst_id: Motor feedback ID (default 0x11)
        adapter_sn: USB-CANFD adapter serial number (None = auto-detect)
        calib_path: Calibration file path (None = use bundled default)
        friction_path: Friction model file path (None = use bundled default)
        auto_enable: Enable motor on init (default True)

    Example::

        with Gripper() as g:
            g.set_position(0)          # close
            g.set_position(100)        # open
            g.set_position(0, force_limit=0.5)  # close with force limit
            g.set_torque(0.3)          # constant grip
    """

    def __init__(
        self,
        can_id: int = 0x01,
        mst_id: int = 0x11,
        adapter_sn: str | None = None,
        calib_path: str | None = None,
        friction_path: str | None = None,
        auto_enable: bool = True,
    ):
        self._dev = DmDevice()
        self._ctrl: GripperController | None = None

        # Open CAN connection
        open_kwargs = dict(nom_baud_hz=1_000_000, dat_baud_hz=5_000_000)
        if adapter_sn:
            open_kwargs["sn"] = adapter_sn
        self._dev.open(**open_kwargs)

        # Create controller
        self._ctrl = GripperController(self._dev, can_id, mst_id)

        # Load calibration
        calib = calib_path or _DEFAULT_CALIB
        if os.path.exists(calib):
            self._ctrl.load_calibration(calib)
        else:
            raise FileNotFoundError(
                f"No calibration file at {calib}. Run gripper.calibrate() first.")

        # Load friction model (optional, enhances smoothness)
        friction = friction_path or _DEFAULT_FRICTION
        if os.path.exists(friction):
            self._ctrl.load_friction(friction)

        # Enable motor
        if auto_enable:
            self._ctrl.enable()

    # ── Core API ──

    def set_position(self, percent: float, force_limit: float | None = None,
                     speed: float = 1.0):
        """Move gripper to target opening.

        Args:
            percent: Opening percentage, 0 = fully closed, 100 = fully open.
            force_limit: If None, pure position mode (ignores obstacles).
                         If >0, force-limited mode — pushes with at most
                         this many Nm, stops when blocked.
            speed: Speed multiplier 0.1 (slow) to 1.0 (fastest safe speed).
        """
        percent = max(0.0, min(100.0, percent))
        speed = max(0.1, min(1.0, speed))

        if force_limit is not None:
            force_limit = abs(force_limit)
            self._ctrl.grip_pct(percent, force_limit)
        else:
            duration = self._ctrl.move_duration / speed
            self._ctrl.move_to_pct(percent, duration=duration)

    def set_torque(self, torque: float):
        """Constant torque mode.

        Args:
            torque: Torque in Nm. Positive = close direction,
                    negative = open direction.
        """
        # Map user convention (positive=close) to motor direction
        angle_sign = 1.0 if self._ctrl._angle_open > self._ctrl._angle_close else -1.0
        self._ctrl.set_torque(-angle_sign * abs(torque) if torque > 0 else angle_sign * abs(torque))

    def stop(self):
        """Stop and hold at current position."""
        self._ctrl.stop()

    def get_state(self) -> dict:
        """Get current gripper state.

        Returns:
            dict with keys:
                position: Opening percentage (0-100)
                torque: Current torque (Nm)
                velocity: Current velocity (rad/s)
                mode: "position", "torque", or "hybrid"
                is_moving: Whether gripper is in motion
                error: Motor error code (1=OK, 0=disabled, 9=undervoltage)
                temperature: Motor temperature (°C)
        """
        s = self._ctrl.get_state()
        return {
            "position": round(s["opening_pct"], 1) if s["opening_pct"] is not None else None,
            "torque": round(s["torque"], 3),
            "velocity": round(s["velocity"], 3),
            "mode": s["mode"],
            "is_moving": abs(s["velocity"]) > 0.1,
            "error": s["error"],
            "temperature": s["t_mos"],
        }

    def wait_until_reached(self, timeout: float = 5.0) -> bool:
        """Block until gripper reaches target or timeout.

        Returns True if target reached, False if timeout.
        """
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            s = self._ctrl.get_state()
            if s["traj_done"] and abs(s["velocity"]) < 0.1:
                return True
            time.sleep(0.02)
        return False

    # ── Calibration ──

    def calibrate(self):
        """Interactive calibration. Finds mechanical limits and sets zero.

        Only needed if the bundled calibration doesn't match your hardware.
        Results are saved and used for subsequent sessions.
        """
        was_enabled = self._ctrl._enabled
        if was_enabled:
            self._ctrl.disable()
        self._ctrl.calibrate_auto()
        # Save to both bundled location and project root
        self._ctrl.save_calibration(_DEFAULT_CALIB)
        proj_root = os.path.dirname(os.path.dirname(_PKG_DIR))
        self._ctrl.save_calibration(os.path.join(proj_root, "gripper_calibration.json"))
        if was_enabled or True:
            self._ctrl.enable()

    # ── Lifecycle ──

    def close(self):
        """Disable motor and close CAN connection."""
        if self._ctrl:
            try:
                self._ctrl.disable()
            except Exception:
                pass
        if self._dev:
            self._dev.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
