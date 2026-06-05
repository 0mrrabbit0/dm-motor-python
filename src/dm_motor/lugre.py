"""
LuGre dynamic friction model and observer.

Estimates and compensates friction in lead screw mechanisms.
The model captures Coulomb, Stribeck, and viscous friction effects
through a single internal state (bristle deflection z).

Reference: Canudas de Wit et al., "A new model for control of
systems with friction", IEEE TAC, 1995.
"""
from __future__ import annotations

import json
import math


class LuGreModel:
    """LuGre friction model with discrete-time observer.

    Parameters:
        fc:     Coulomb friction (Nm) — friction at high speed
        fs:     Static friction (Nm) — peak friction at zero velocity
        vs:     Stribeck velocity (rad/s) — transition speed
        sigma0: Bristle stiffness (Nm/rad) — pre-sliding stiffness
        sigma1: Bristle damping (Nm·s/rad) — micro-damping
        fv:     Viscous friction coefficient (Nm·s/rad)
    """

    def __init__(self, fc: float = 0.1, fs: float = 0.15, vs: float = 1.0,
                 sigma0: float = 1000.0, sigma1: float = 1.0, fv: float = 0.01):
        self.fc = fc
        self.fs = fs
        self.vs = vs
        self.sigma0 = sigma0
        self.sigma1 = sigma1
        self.fv = fv

        self._z = 0.0           # bristle deflection estimate
        self._v_filtered = 0.0  # low-pass filtered velocity
        self._vel_alpha = 0.3   # EMA alpha for velocity filter (~10Hz at 200Hz)

    def reset(self):
        self._z = 0.0
        self._v_filtered = 0.0

    def stribeck(self, v: float) -> float:
        """Stribeck function g(v): steady-state friction vs velocity."""
        return self.fc + (self.fs - self.fc) * math.exp(-(v / self.vs) ** 2)

    def step(self, v_raw: float, dt: float = 0.005) -> float:
        """One observer step. Returns estimated friction torque.

        Args:
            v_raw: measured velocity (rad/s), can be noisy
            dt: time step (default 5ms = 200Hz)

        Returns:
            F_friction: estimated friction torque (Nm),
                        positive = resists positive velocity
        """
        # Low-pass filter velocity to reduce encoder noise
        self._v_filtered += self._vel_alpha * (v_raw - self._v_filtered)
        v = self._v_filtered

        # Stribeck function
        g_v = self.stribeck(v)

        # Bristle dynamics: dz/dt = v - σ0 * |v| / g(v) * z
        abs_v = abs(v)
        if abs_v < 1e-6:
            dz = 0.0
        else:
            dz = v - self.sigma0 * abs_v / g_v * self._z

        # Euler integration
        self._z += dz * dt

        # Clamp z to physical bounds to prevent divergence
        z_max = g_v / self.sigma0
        self._z = max(-z_max, min(z_max, self._z))

        # Friction force: F = σ0*z + σ1*dz + Fv*v
        f_friction = self.sigma0 * self._z + self.sigma1 * dz + self.fv * v

        return f_friction

    def steady_state_friction(self, v: float) -> float:
        """Analytical steady-state friction at constant velocity."""
        g_v = self.stribeck(v)
        sign_v = 1.0 if v >= 0 else -1.0
        return g_v * sign_v + self.fv * v

    def save(self, path: str):
        data = {
            "fc": self.fc, "fs": self.fs, "vs": self.vs,
            "sigma0": self.sigma0, "sigma1": self.sigma1, "fv": self.fv,
        }
        with open(path, "w") as f:
            json.dump(data, f, indent=2)

    @classmethod
    def load(cls, path: str) -> LuGreModel:
        with open(path) as f:
            data = json.load(f)
        return cls(**data)
