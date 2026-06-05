"""
Force-position hybrid gripper controller for DaMiao DM4310.

Mechanical structure: motor → lead screw → sleeve → linkage → jaws.

Uses minimum-jerk trajectory shaping + friction feedforward to eliminate
stick-slip oscillation on the lead screw mechanism. The motor's internal
10kHz PD loop handles fast damping; the 1kHz software loop handles
trajectory planning and friction compensation.

Three control modes:
  POSITION — move jaws to target opening (mm/%%/rad)
  TORQUE   — constant grip torque
  HYBRID   — move to target, stop/comply when force exceeds limit
"""
from __future__ import annotations

import json
import math
import os
import threading
import time
from enum import Enum
from typing import Callable

from dm_motor.sdk import DmDevice
from dm_motor.lugre import LuGreModel


class GripperMode(Enum):
    POSITION = "position"
    TORQUE = "torque"
    HYBRID = "hybrid"


class GripperState(Enum):
    IDLE = "idle"
    TRACKING = "tracking"
    HOLDING = "holding"
    COMPLYING = "complying"


# --- Defaults ---
_DEFAULT_KP_TRACK = 5.0   # software PD gain — lower prevents undervoltage on lead screw
_DEFAULT_KD_TRACK = 2.0
_DEFAULT_KP_HOLD = 2.0
_DEFAULT_KD_HOLD = 1.5
_DEFAULT_KD_COMPLY = 2.0
_DEFAULT_KD_TORQUE = 0.5

_DEFAULT_TAU_FRICTION = 0.3   # Coulomb friction compensation (Nm)
_DEFAULT_MOVE_DURATION = 1.0  # full travel duration (s) — validated smooth at 0.6s/60%

_DEFAULT_HYSTERESIS = 0.8
_DEFAULT_TAU_SAFETY = 3.0  # lead screw: higher torque causes undervoltage on stall

_LOOP_PERIOD_S = 0.005  # 200Hz — validated stable for lead screw mechanisms
_TEMP_MOS_MAX = 80
_TEMP_ROTOR_MAX = 100
_FEEDBACK_TIMEOUT_S = 0.5


# --- Minimum-jerk trajectory generator ---

class MinJerkTrajectory:
    """Generates smooth position/velocity profiles with zero accel at endpoints."""

    def __init__(self, q0: float, qf: float, duration: float):
        self.q0 = q0
        self.qf = qf
        self.duration = max(duration, 0.05)
        self.t_start = time.monotonic()

    def sample(self) -> tuple[float, float, bool]:
        """Returns (q_des, dq_des, done)."""
        t = time.monotonic() - self.t_start
        if t >= self.duration:
            return self.qf, 0.0, True

        s = t / self.duration
        # Minimum-jerk: s(t) = 10s³ - 15s⁴ + 6s⁵
        phase = 10 * s**3 - 15 * s**4 + 6 * s**5
        # ds/dt for velocity feedforward
        dphase = (30 * s**2 - 60 * s**3 + 30 * s**4) / self.duration

        q_des = self.q0 + (self.qf - self.q0) * phase
        dq_des = (self.qf - self.q0) * dphase
        return q_des, dq_des, False


class GripperController:
    """High-level gripper controller with trajectory shaping."""

    def __init__(
        self,
        device: DmDevice,
        can_id: int,
        mst_id: int,
        *,
        kp_track: float = _DEFAULT_KP_TRACK,
        kd_track: float = _DEFAULT_KD_TRACK,
        kp_hold: float = _DEFAULT_KP_HOLD,
        kd_hold: float = _DEFAULT_KD_HOLD,
        kd_comply: float = _DEFAULT_KD_COMPLY,
        kd_torque: float = _DEFAULT_KD_TORQUE,
        tau_friction: float = _DEFAULT_TAU_FRICTION,
        move_duration: float = _DEFAULT_MOVE_DURATION,
        hysteresis: float = _DEFAULT_HYSTERESIS,
        tau_safety: float = _DEFAULT_TAU_SAFETY,
        on_state_change: Callable[[GripperState, GripperState], None] | None = None,
    ):
        self._dev = device
        self._can_id = can_id
        self._mst_id = mst_id

        self.kp_track = kp_track
        self.kd_track = kd_track
        self.kp_hold = kp_hold
        self.kd_hold = kd_hold
        self.kd_comply = kd_comply
        self.kd_torque = kd_torque
        self.tau_friction = tau_friction
        self.move_duration = move_duration
        self.hysteresis = hysteresis
        self.tau_safety = tau_safety
        self._on_state_change = on_state_change

        # Calibration
        self._angle_close = None
        self._angle_open = None
        self._stroke_mm = None
        self._calibrated = False

        # Control state
        self._mode = GripperMode.POSITION
        self._state = GripperState.IDLE
        self._target_angle = 0.0
        self._target_tau = 0.0
        self._force_limit = 5.0
        self._hold_angle = 0.0

        # Trajectory (None when holding at target)
        self._traj: MinJerkTrajectory | None = None
        self._traj_done = True

        self._running = False
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._enabled = False

        self._last_fb_time = 0.0
        self._pos = 0.0
        self._vel = 0.0
        self._tau = 0.0
        self._err = 0
        self._t_mos = 0
        self._t_rotor = 0
        self._tau_cmd_filtered = 0.0  # low-pass filtered torque command
        self._traj_end_time = 0.0     # time when trajectory finished
        self._kd_motion = 0.5         # Kd during motion

        # Stall detection for hybrid mode
        self._stall_pos = 0.0
        self._stall_time = 0.0
        self._stalled = False

        # LuGre friction compensation (loaded from gripper_friction.json)
        self._lugre: LuGreModel | None = None

    # ── Calibration ──

    def calibrate_auto(self, calib_tau: float = 0.4,
                       stall_confirm_s: float = 0.3,
                       pos_change_threshold: float = 0.01):
        """Auto calibration via stall detection."""
        print("\n=== Gripper Auto-Calibration ===")

        self._stop_loop()
        for _ in range(5):
            self._dev.enable(self._can_id)
            time.sleep(0.005)
        self._enabled = True
        time.sleep(0.2)

        for _ in range(100):
            self._dev.control_mit(
                self._can_id, kp=0, kd=1.0, q=0, dq=0, tau=0)
            time.sleep(0.005)

        def _drive_until_stall(tau_cmd: float, label: str) -> float:
            last_check_pos = None
            last_check_time = time.monotonic()
            stable_since = None

            while True:
                self._dev.control_mit(
                    self._can_id, kp=0, kd=0.5, q=0, dq=0, tau=tau_cmd)
                time.sleep(0.005)
                fb = self._dev.get_feedback(self._mst_id)
                if not fb:
                    continue
                pos = fb["q"]
                if fb["err"] not in (0, 1):
                    print(f"  {label}: {pos:+.4f} rad (motor err={fb['err']})")
                    time.sleep(0.1)
                    for _ in range(5):
                        self._dev.enable(self._can_id)
                        time.sleep(0.005)
                    time.sleep(0.1)
                    return pos
                if fb["err"] == 0:
                    self._dev.enable(self._can_id)
                    time.sleep(0.005)
                    stable_since = None
                    continue
                now = time.monotonic()
                if now - last_check_time >= 0.1:
                    if last_check_pos is not None:
                        if abs(pos - last_check_pos) < pos_change_threshold:
                            if stable_since is None:
                                stable_since = now
                            elif now - stable_since >= stall_confirm_s:
                                print(f"  {label}: {pos:+.4f} rad (stalled)")
                                return pos
                        else:
                            stable_since = None
                    last_check_pos = pos
                    last_check_time = now

        # Find both limits
        print("\n[1/2] Finding limit A (-tau)...")
        angle_neg = _drive_until_stall(-calib_tau, "Limit A")

        for _ in range(50):
            self._dev.control_mit(self._can_id, kp=0, kd=1.0, q=0, dq=0, tau=0)
            time.sleep(0.01)

        print("[2/2] Finding limit B (+tau)...")
        angle_pos = _drive_until_stall(+calib_tau, "Limit B")

        for _ in range(50):
            self._dev.control_mit(self._can_id, kp=0, kd=1.0, q=0, dq=0, tau=0)
            time.sleep(0.01)
        for _ in range(5):
            self._dev.disable(self._can_id)
            time.sleep(0.005)
        self._enabled = False

        travel_rad = abs(angle_pos - angle_neg)
        print(f"\n  Limit A: {angle_neg:+.4f} rad")
        print(f"  Limit B: {angle_pos:+.4f} rad")
        print(f"  Travel:  {travel_rad:.4f} rad ({travel_rad / 6.2832:.2f} turns)")

        print("\n  Gripper is now at Limit B (the +tau end).")
        answer = input(">> Is the gripper currently OPEN or CLOSED? (o/c): ").strip().lower()
        if answer.startswith("o"):
            self._angle_open, self._angle_close = angle_pos, angle_neg
        else:
            self._angle_open, self._angle_close = angle_neg, angle_pos

        # Set zero at closed position so angles are repeatable across reboots
        # Re-enable to send set_zero command
        for _ in range(5):
            self._dev.enable(self._can_id)
            time.sleep(0.005)
        time.sleep(0.1)
        # Move to closed position briefly to set zero there
        for _ in range(100):
            self._dev.control_mit(
                self._can_id, kp=5, kd=2, q=self._angle_close, dq=0, tau=0)
            time.sleep(0.005)
        time.sleep(0.3)
        self._dev.set_zero(self._can_id)
        time.sleep(0.1)
        # Persist zero to motor flash — survives power cycles
        self._dev.save_params(self._can_id)
        time.sleep(0.2)
        for _ in range(5):
            self._dev.disable(self._can_id)
            time.sleep(0.005)
        self._enabled = False

        # Recalculate angles relative to new zero (closed = 0)
        travel = self._angle_open - self._angle_close
        self._angle_close = 0.0
        self._angle_open = travel

        print(f"  Zero saved to motor flash (persists across power cycles).")
        print(f"  Closed: {self._angle_close:+.4f} rad (= 0)")
        print(f"  Open:   {self._angle_open:+.4f} rad")

        try:
            s = input(">> Jaw opening at fully open (mm), or Enter to skip: ").strip()
            self._stroke_mm = float(s) if s else None
        except ValueError:
            self._stroke_mm = None

        self._calibrated = True
        print("Calibration complete.\n")

    def auto_home(self, path: str = None):
        """Auto-home on startup: find closed limit, set zero, apply saved travel.

        Only needs the travel distance from a previous full calibration.
        Takes ~3-5 seconds. No user interaction required.
        """
        if path is None:
            path = self._default_calib_path()
        if not os.path.exists(path):
            raise RuntimeError(
                "No calibration file. Run calibrate_auto() first to establish travel distance.")

        with open(path) as f:
            data = json.load(f)
        travel = data["angle_open"] - data["angle_close"]
        stroke_mm = data.get("stroke_mm")

        print("[AUTO-HOME] Finding closed limit...")
        self._stop_loop()

        # Enable motor
        for _ in range(5):
            self._dev.enable(self._can_id)
            time.sleep(0.005)
        self._enabled = True
        time.sleep(0.2)

        # Warm up feedback
        for _ in range(50):
            self._dev.control_mit(
                self._can_id, kp=0, kd=1.0, q=0, dq=0, tau=0)
            time.sleep(0.005)

        # Drive toward closed limit with small torque
        # Determine direction: close is at the smaller-travel end
        # If travel < 0, close is at higher angle → drive positive
        # If travel > 0, close is at lower angle → drive negative
        close_tau = 0.4 if travel < 0 else -0.4

        last_pos = None
        last_check = time.monotonic()
        stable_since = None

        while True:
            self._dev.control_mit(
                self._can_id, kp=0, kd=0.5, q=0, dq=0, tau=close_tau)
            time.sleep(0.005)
            fb = self._dev.get_feedback(self._mst_id)
            if not fb:
                continue
            if fb["err"] not in (0, 1):
                # Hit limit hard enough to trigger error → use this as limit
                pos = fb["q"]
                print(f"  Closed limit: {pos:+.4f} rad (motor err={fb['err']})")
                time.sleep(0.1)
                for _ in range(5):
                    self._dev.enable(self._can_id)
                    time.sleep(0.005)
                time.sleep(0.1)
                break
            if fb["err"] == 0:
                self._dev.enable(self._can_id)
                time.sleep(0.005)
                stable_since = None
                continue
            pos = fb["q"]
            now = time.monotonic()
            if now - last_check >= 0.1:
                if last_pos is not None and abs(pos - last_pos) < 0.01:
                    if stable_since is None:
                        stable_since = now
                    elif now - stable_since >= 0.3:
                        print(f"  Closed limit: {pos:+.4f} rad (stalled)")
                        break
                else:
                    stable_since = None
                last_pos = pos
                last_check = now

        # Set zero at closed position and persist to flash
        for _ in range(50):
            self._dev.control_mit(
                self._can_id, kp=0, kd=1.0, q=0, dq=0, tau=0)
            time.sleep(0.005)
        self._dev.set_zero(self._can_id)
        time.sleep(0.1)
        self._dev.save_params(self._can_id)
        time.sleep(0.2)

        # Disable
        for _ in range(5):
            self._dev.disable(self._can_id)
            time.sleep(0.005)
        self._enabled = False

        self._angle_close = 0.0
        self._angle_open = travel
        self._stroke_mm = stroke_mm
        self._calibrated = True

        # Save updated calibration
        self.save_calibration(path)

        print(f"[AUTO-HOME] Done. close=0, open={travel:+.4f} rad")

    def save_calibration(self, path: str = None):
        if not self._calibrated:
            raise RuntimeError("Not calibrated")
        if path is None:
            path = self._default_calib_path()
        data = {"angle_close": self._angle_close, "angle_open": self._angle_open,
                "stroke_mm": self._stroke_mm}
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
        print(f"Saved to {path}")

    def load_calibration(self, path: str = None):
        if path is None:
            path = self._default_calib_path()
        with open(path) as f:
            data = json.load(f)
        self._angle_close = data["angle_close"]
        self._angle_open = data["angle_open"]
        self._stroke_mm = data.get("stroke_mm")
        self._calibrated = True
        print(f"Calibration loaded: close={self._angle_close:.4f}, open={self._angle_open:.4f}")

    def load_friction(self, path: str = None):
        if path is None:
            pkg_dir = os.path.dirname(os.path.abspath(__file__))
            path = os.path.join(os.path.dirname(os.path.dirname(pkg_dir)),
                                "gripper_friction.json")
        if not os.path.exists(path):
            return False
        self._lugre = LuGreModel.load(path)
        print(f"Friction model loaded: Fc={self._lugre.fc:.4f}, Fs={self._lugre.fs:.4f}, "
              f"vs={self._lugre.vs:.2f}")
        return True

    def _default_calib_path(self):
        pkg_dir = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(os.path.dirname(os.path.dirname(pkg_dir)),
                            "gripper_calibration.json")

    def _mm_to_rad(self, mm: float) -> float:
        if not self._calibrated or self._stroke_mm is None:
            raise RuntimeError("Calibration with stroke_mm required")
        ratio = (self._angle_open - self._angle_close) / self._stroke_mm
        return self._angle_close + mm * ratio

    def _rad_to_mm(self, rad: float) -> float | None:
        if not self._calibrated or self._stroke_mm is None:
            return None
        return (rad - self._angle_close) / (self._angle_open - self._angle_close) * self._stroke_mm

    def _pct_to_rad(self, pct: float) -> float:
        if not self._calibrated:
            raise RuntimeError("Calibration required")
        return self._angle_close + (self._angle_open - self._angle_close) * pct / 100.0

    def _rad_to_pct(self, rad: float) -> float | None:
        if not self._calibrated:
            return None
        return (rad - self._angle_close) / (self._angle_open - self._angle_close) * 100.0

    # ── Lifecycle ──

    def enable(self):
        for _ in range(5):
            self._dev.enable(self._can_id)
            time.sleep(0.005)
        self._enabled = True
        time.sleep(0.1)
        for _ in range(100):
            self._dev.control_mit(
                self._can_id, kp=0, kd=self.kd_track, q=0, dq=0, tau=0)
            time.sleep(_LOOP_PERIOD_S)
        fb = self._dev.get_feedback(self._mst_id)
        if fb:
            self._pos = fb["q"]
            self._target_angle = fb["q"]
        self._mode = GripperMode.POSITION
        self._traj = None
        self._traj_done = True
        self._ensure_loop()

    def disable(self):
        self._stop_loop()
        if self._enabled:
            for _ in range(10):
                self._dev.control_mit(
                    self._can_id, kp=0, kd=self.kd_track, q=0, dq=0, tau=0)
                time.sleep(0.01)
            for _ in range(5):
                self._dev.disable(self._can_id)
                time.sleep(0.005)
            self._enabled = False
        self._set_state(GripperState.IDLE)

    def _stop_loop(self):
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def __enter__(self):
        self.enable()
        return self

    def __exit__(self, *exc):
        self.disable()

    # ── High-level commands ──

    def move_to_angle(self, angle_rad: float, duration: float | None = None):
        """Position mode: smooth move to target angle."""
        angle_rad = self._clamp_angle(angle_rad)
        dur = duration or self._compute_duration(angle_rad)
        with self._lock:
            self._mode = GripperMode.POSITION
            self._target_angle = angle_rad
            self._traj = MinJerkTrajectory(self._pos, angle_rad, dur)
            self._traj_done = False
        self._ensure_loop()

    def move_to_mm(self, opening_mm: float, duration: float | None = None):
        self.move_to_angle(self._mm_to_rad(opening_mm), duration)

    def move_to_pct(self, pct: float, duration: float | None = None):
        pct = max(0.0, min(100.0, pct))
        self.move_to_angle(self._pct_to_rad(pct), duration)

    def set_torque(self, torque: float):
        torque = max(-self.tau_safety, min(self.tau_safety, torque))
        with self._lock:
            self._mode = GripperMode.TORQUE
            self._target_tau = torque
            self._traj = None
            self._traj_done = True
        self._ensure_loop()

    def grip(self, angle_rad: float, force_limit: float, duration: float | None = None):
        """Hybrid mode: move to angle with force limit (always positive Nm)."""
        angle_rad = self._clamp_angle(angle_rad)
        force_limit = max(0.01, min(abs(force_limit), self.tau_safety))
        with self._lock:
            self._mode = GripperMode.HYBRID
            self._target_angle = angle_rad
            self._force_limit = force_limit
            self._set_state(GripperState.TRACKING)
        self._ensure_loop()

    def grip_pct(self, pct: float, force_limit: float, duration: float | None = None):
        pct = max(0.0, min(100.0, pct))
        self.grip(self._pct_to_rad(pct), force_limit, duration)

    def open_gripper(self, force_limit: float | None = None):
        if not self._calibrated:
            raise RuntimeError("Calibrate first")
        if force_limit is not None:
            self.grip(self._angle_open, force_limit)
        else:
            self.move_to_angle(self._angle_open)

    def close_gripper(self, force_limit: float | None = None):
        if not self._calibrated:
            raise RuntimeError("Calibrate first")
        if force_limit is not None:
            self.grip(self._angle_close, force_limit)
        else:
            self.move_to_angle(self._angle_close)

    def stop(self):
        with self._lock:
            self._mode = GripperMode.POSITION
            self._target_angle = self._pos
            self._traj = None
            self._traj_done = True

    def get_state(self) -> dict:
        with self._lock:
            is_holding = self._state in (GripperState.HOLDING, GripperState.COMPLYING)
            loop_alive = self._thread.is_alive() if self._thread else False
            return dict(
                loop_alive=loop_alive,
                mode=self._mode.value,
                state=self._state.value,
                angle_rad=self._pos,
                opening_mm=self._rad_to_mm(self._pos),
                opening_pct=self._rad_to_pct(self._pos),
                velocity=self._vel,
                torque=self._tau,
                target_angle=self._target_angle,
                hold_angle=self._hold_angle if is_holding else None,
                target_torque=self._target_tau if self._mode == GripperMode.TORQUE else None,
                force_limit=self._force_limit if self._mode == GripperMode.HYBRID else None,
                traj_done=self._traj_done,
                error=self._err,
                t_mos=self._t_mos,
                t_rotor=self._t_rotor,
                enabled=self._enabled,
                calibrated=self._calibrated,
            )

    # ── Control loop ──

    def _ensure_loop(self):
        with self._lock:
            if self._running:
                return
            self._running = True
        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()

    def _control_loop(self):
        self._last_fb_time = time.monotonic()

        while self._running:
            t_start = time.monotonic()
            fb = self._dev.get_feedback(self._mst_id)

            if fb:
                self._last_fb_time = t_start
                self._pos = fb["q"]
                self._vel = fb["dq"]
                self._tau = fb["tau"]
                self._err = fb["err"]
                self._t_mos = fb["t_mos"]
                self._t_rotor = fb["t_rotor"]
                if self._err == 0 and self._enabled:
                    self._dev.enable(self._can_id)
                    time.sleep(0.002)
            elif t_start - self._last_fb_time > _FEEDBACK_TIMEOUT_S:
                self._handle_fault("feedback timeout")
                break

            if not self._check_safety():
                break

            with self._lock:
                mode = self._mode
                target_angle = self._target_angle
                target_tau = self._target_tau
                force_limit = self._force_limit
                traj = self._traj

            if mode == GripperMode.POSITION:
                self._step_position(target_angle, traj)
            elif mode == GripperMode.TORQUE:
                self._step_torque(target_tau)
            elif mode == GripperMode.HYBRID:
                self._step_hybrid(target_angle, force_limit, traj)

            elapsed = time.monotonic() - t_start
            sleep_time = _LOOP_PERIOD_S - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _filter_tau(self, tau_raw: float, alpha: float = 0.15) -> float:
        """Exponential moving average on torque command.
        alpha=0.15 at 200Hz ≈ 5Hz cutoff, removes high-freq torque steps."""
        self._tau_cmd_filtered += alpha * (tau_raw - self._tau_cmd_filtered)
        return self._tau_cmd_filtered

    def _send_mit(self, kp: float, kd: float, q: float = 0,
                  dq: float = 0, tau: float = 0):
        self._dev.control_mit(self._can_id, kp=kp, kd=kd, q=q, dq=dq, tau=tau)

    def _friction_ff(self, dq_des: float) -> float:
        """Coulomb friction feedforward with smooth sign transition."""
        if abs(dq_des) < 0.001:
            return 0.0
        return self.tau_friction * math.tanh(dq_des / 0.5)

    def _step_position(self, target: float, traj: MinJerkTrajectory | None):
        self._set_state(GripperState.TRACKING)
        if traj and not self._traj_done:
            q_des, dq_des, done = traj.sample()
            if done:
                with self._lock:
                    self._traj_done = True
                self._traj_end_time = time.monotonic()
                q_des = target
                dq_des = 0.0
            kd = self._kd_motion
        else:
            q_des = target
            dq_des = 0.0
            # Smooth Kd ramp: 0.5 → 2.0 over 300ms after trajectory ends
            t_since_end = time.monotonic() - self._traj_end_time if self._traj_end_time > 0 else 10.0
            ramp = min(1.0, t_since_end / 0.3)
            kd = self._kd_motion + (self.kd_track - self._kd_motion) * ramp

        error = q_des - self._pos
        tau_pd = self.kp_track * error

        # LuGre friction feedforward compensation
        if self._lugre is not None:
            f_friction = self._lugre.step(self._vel, _LOOP_PERIOD_S)
            tau_raw = tau_pd + f_friction
        else:
            tau_raw = tau_pd

        tau_raw = max(-self.tau_safety, min(self.tau_safety, tau_raw))
        tau_cmd = self._filter_tau(tau_raw)
        self._send_mit(0, kd, dq=dq_des, tau=tau_cmd)

    def _step_torque(self, target_tau: float):
        self._set_state(GripperState.TRACKING)
        self._send_mit(0, self.kd_torque, tau=target_tau)

    def _step_hybrid(self, target_angle: float, force_limit: float,
                     traj: MinJerkTrajectory | None):
        """Force-limited position control.

        Pushes toward target with at most ±force_limit torque.
        Stops when blocked, resumes when released.
        """
        self._set_state(GripperState.TRACKING)
        error = target_angle - self._pos
        tau = self.kp_track * error
        tau = max(-force_limit, min(force_limit, tau))
        self._send_mit(0, self.kd_track, tau=tau)

    # ── Safety ──

    def _check_safety(self) -> bool:
        if self._t_mos > _TEMP_MOS_MAX:
            self._handle_fault(f"MOS overtemp: {self._t_mos}°C")
            return False
        if self._t_rotor > _TEMP_ROTOR_MAX:
            self._handle_fault(f"rotor overtemp: {self._t_rotor}°C")
            return False
        if self._err not in (0, 1):
            self._handle_fault(f"motor error code: {self._err}")
            return False
        return True

    def _handle_fault(self, reason: str):
        print(f"[Gripper FAULT] {reason} — disabling motor", flush=True)
        self._running = False
        try:
            self._dev.control_mit(
                self._can_id, kp=0, kd=self.kd_track, q=0, dq=0, tau=0)
            time.sleep(0.02)
            self._dev.disable(self._can_id)
        except Exception:
            pass
        self._enabled = False
        self._set_state(GripperState.IDLE)

    # ── Helpers ──

    def _set_state(self, new: GripperState):
        old = self._state
        if old != new:
            self._state = new
            if self._on_state_change:
                try:
                    self._on_state_change(old, new)
                except Exception:
                    pass

    def _clamp_angle(self, angle: float) -> float:
        if not self._calibrated:
            return angle
        lo = min(self._angle_close, self._angle_open)
        hi = max(self._angle_close, self._angle_open)
        return max(lo, min(hi, angle))

    def _compute_duration(self, target: float) -> float:
        """Scale move duration by distance, capping peak velocity."""
        dist = abs(target - self._pos)
        if dist < 0.01:
            return 0.1
        if not self._calibrated:
            return self.move_duration
        total_travel = abs(self._angle_open - self._angle_close)
        if total_travel < 0.01:
            return self.move_duration
        # Proportional scaling
        dur_proportional = self.move_duration * dist / total_travel
        # Min-jerk peak velocity = dist * 1.875 / duration
        # Cap peak velocity at 14 rad/s — prevents settling oscillation on short moves
        dur_vel_limit = dist * 1.875 / 14.0
        return max(dur_proportional, dur_vel_limit, 0.15)
