"""
Gripper oscillation diagnostic tool.

Collects time-series data under different control conditions,
analyzes for oscillation (FFT, LPC, step response), and
determines root cause before attempting any fix.

Outputs: CSV data files + text analysis report.
"""
import csv
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from dm_motor import DmDevice

ADAPTER_SN = "14AA044B241402B10DDBDAFE448040BB"
CAN_ID = 0x01
MST_ID = 0x11
OUT_DIR = os.path.join(os.path.dirname(__file__), "..", "diag_data")


def ensure_dir():
    os.makedirs(OUT_DIR, exist_ok=True)


def collect_data(dev, label, duration_s, cmd_fn, rate_hz=200):
    """Collect time-series data while executing cmd_fn each cycle."""
    data = []
    period = 1.0 / rate_hz
    t0 = time.monotonic()

    while time.monotonic() - t0 < duration_s:
        t = time.monotonic() - t0
        cmd_fn(t)
        fb = dev.get_feedback(MST_ID)
        if fb:
            data.append({
                "t": round(t, 4),
                "pos": fb["q"],
                "vel": fb["dq"],
                "tau": fb["tau"],
                "err": fb["err"],
            })
        time.sleep(period)

    path = os.path.join(OUT_DIR, f"{label}.csv")
    if data:
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["t", "pos", "vel", "tau", "err"])
            w.writeheader()
            w.writerows(data)
    print(f"  [{label}] {len(data)} samples -> {path}")
    return data


def analyze_oscillation(data, label):
    """Analyze time-series for oscillation characteristics."""
    if len(data) < 50:
        return {"label": label, "error": "insufficient data"}

    pos = [d["pos"] for d in data]
    vel = [d["vel"] for d in data]
    tau = [d["tau"] for d in data]
    t = [d["t"] for d in data]

    n = len(pos)
    dt = (t[-1] - t[0]) / (n - 1) if n > 1 else 0.005

    # --- Basic stats ---
    pos_mean = sum(pos) / n
    pos_std = math.sqrt(sum((p - pos_mean)**2 for p in pos) / n)
    vel_rms = math.sqrt(sum(v**2 for v in vel) / n)
    tau_rms = math.sqrt(sum(t**2 for t in tau) / n)
    pos_p2p = max(pos) - min(pos)

    # --- FFT on position signal (detrended) ---
    pos_detrended = [p - pos_mean for p in pos]
    fft_mag = _fft_magnitude(pos_detrended)
    freqs = [i / (n * dt) for i in range(len(fft_mag))]

    # Find dominant frequency (skip DC)
    if len(fft_mag) > 2:
        peak_idx = max(range(1, len(fft_mag)), key=lambda i: fft_mag[i])
        peak_freq = freqs[peak_idx]
        peak_amp = fft_mag[peak_idx] * 2 / n
    else:
        peak_freq = 0
        peak_amp = 0

    # --- LPC oscillation detection ---
    is_oscillating_lpc = _lpc_oscillation_detect(pos_detrended)

    # --- Zero-crossing count on velocity ---
    zc = 0
    for i in range(1, len(vel)):
        if vel[i-1] * vel[i] < 0:
            zc += 1
    zc_rate = zc / (t[-1] - t[0]) if t[-1] > t[0] else 0

    # --- Classify ---
    is_oscillating = (
        is_oscillating_lpc
        or (pos_p2p > 0.03 and zc_rate > 4)
        or (peak_amp > 0.01 and peak_freq > 1)
    )

    result = {
        "label": label,
        "n_samples": n,
        "duration_s": round(t[-1] - t[0], 2),
        "pos_mean": round(pos_mean, 4),
        "pos_std": round(pos_std, 4),
        "pos_p2p": round(pos_p2p, 4),
        "vel_rms": round(vel_rms, 4),
        "tau_rms": round(tau_rms, 4),
        "fft_peak_freq_hz": round(peak_freq, 2),
        "fft_peak_amp_rad": round(peak_amp, 5),
        "vel_zero_crossings": zc,
        "vel_zc_rate_hz": round(zc_rate, 2),
        "lpc_oscillating": is_oscillating_lpc,
        "is_oscillating": is_oscillating,
    }
    return result


def _fft_magnitude(signal):
    """Simple DFT magnitude (no numpy dependency)."""
    n = len(signal)
    half = n // 2
    mag = []
    for k in range(half):
        re = sum(signal[j] * math.cos(2 * math.pi * k * j / n) for j in range(n))
        im = sum(signal[j] * math.sin(2 * math.pi * k * j / n) for j in range(n))
        mag.append(math.sqrt(re**2 + im**2))
    return mag


def _lpc_oscillation_detect(signal, order=10, threshold=0.95):
    """LPC-based oscillation detection (Sharma et al. 2020).

    Fits autoregressive model, checks if any root of the characteristic
    polynomial is close to the unit circle (indicating sustained oscillation).
    """
    n = len(signal)
    if n < order * 3:
        return False

    # Levinson-Durbin to estimate AR coefficients
    r = [sum(signal[i] * signal[i+k] for i in range(n-k)) / n
         for k in range(order + 1)]

    if abs(r[0]) < 1e-12:
        return False

    a = [0.0] * (order + 1)
    a[0] = 1.0
    err = r[0]

    for m in range(1, order + 1):
        lam = sum(a[j] * r[m-j] for j in range(m))
        if abs(err) < 1e-12:
            break
        km = -lam / err

        a_new = list(a)
        for j in range(1, m):
            a_new[j] = a[j] + km * a[m-j]
        a_new[m] = km
        a = a_new
        err *= (1 - km * km)

    # Find roots of polynomial 1 + a1*z^-1 + a2*z^-2 + ...
    # Companion matrix method (simplified: just check radius of roots)
    coeffs = [a[i] for i in range(1, order + 1)]
    roots = _poly_roots_approximate(coeffs)

    max_radius = max(abs(r) for r in roots) if roots else 0
    return max_radius > threshold


def _poly_roots_approximate(coeffs):
    """Approximate roots of z^n + c1*z^(n-1) + ... + cn using Durand-Kerner."""
    n = len(coeffs)
    if n == 0:
        return []

    # Initial guesses on unit circle
    roots = [0.4 * complex(math.cos(2*math.pi*k/n + 0.1),
                            math.sin(2*math.pi*k/n + 0.1))
             for k in range(n)]

    for _ in range(100):
        for i in range(n):
            z = roots[i]
            # Evaluate polynomial: z^n + c1*z^(n-1) + ... + cn
            pz = z**n
            for j, c in enumerate(coeffs):
                pz += c * z**(n-1-j)
            # Product of (z_i - z_j) for j != i
            denom = 1.0
            for j in range(n):
                if j != i:
                    diff = roots[i] - roots[j]
                    if abs(diff) < 1e-15:
                        diff = 1e-15
                    denom *= diff
            roots[i] -= pz / denom

    return roots


def print_report(results):
    """Print diagnostic report."""
    print("\n" + "=" * 70)
    print("  OSCILLATION DIAGNOSTIC REPORT")
    print("=" * 70)

    for r in results:
        osc = "!! OSCILLATING !!" if r.get("is_oscillating") else "OK (stable)"
        print(f"\n  [{r['label']}] {osc}")
        print(f"    pos: mean={r['pos_mean']:+.4f} std={r['pos_std']:.4f} "
              f"p2p={r['pos_p2p']:.4f} rad")
        print(f"    vel: rms={r['vel_rms']:.4f} rad/s  "
              f"zero-crossings={r['vel_zero_crossings']} ({r['vel_zc_rate_hz']:.1f} Hz)")
        print(f"    tau: rms={r['tau_rms']:.4f} Nm")
        print(f"    FFT: peak={r['fft_peak_freq_hz']:.1f} Hz, "
              f"amp={r['fft_peak_amp_rad']:.5f} rad")
        print(f"    LPC: {'oscillating' if r['lpc_oscillating'] else 'stable'}")

    print("\n" + "=" * 70)

    # Recommendation
    osc_tests = [r for r in results if r.get("is_oscillating")]
    if not osc_tests:
        print("  All tests stable. No oscillation detected.")
    else:
        freqs = [r["fft_peak_freq_hz"] for r in osc_tests if r["fft_peak_freq_hz"] > 0]
        amps = [r["pos_p2p"] for r in osc_tests]
        print(f"  Oscillating tests: {[r['label'] for r in osc_tests]}")
        if freqs:
            print(f"  Frequency range: {min(freqs):.1f} - {max(freqs):.1f} Hz")
        print(f"  Amplitude range: {min(amps):.4f} - {max(amps):.4f} rad")

        # Root cause analysis
        hold_osc = any("hold" in r["label"] for r in osc_tests)
        traj_osc = any("traj" in r["label"] for r in osc_tests)
        if hold_osc:
            print("\n  ROOT CAUSE: Oscillation during HOLD (Kp>0 at rest)")
            print("  FIX: Use Kp=0 during hold. Let friction hold position.")
        if traj_osc and not hold_osc:
            print("\n  ROOT CAUSE: Oscillation during trajectory tracking")
            print("  FIX: Slow down trajectory or reduce Kp.")

    print("=" * 70)


def main():
    ensure_dir()
    dev = DmDevice()
    results = []

    try:
        print("[INIT] Opening CAN-FD ...")
        dev.open(nom_baud_hz=1_000_000, dat_baud_hz=5_000_000, sn=ADAPTER_SN)

        print("[INIT] Enabling motor ...")
        for _ in range(5):
            dev.enable(CAN_ID)
            time.sleep(0.005)
        time.sleep(0.3)

        # Warm up feedback
        for _ in range(100):
            dev.control_mit(CAN_ID, kp=0, kd=1.0, q=0, dq=0, tau=0)
            time.sleep(0.005)

        fb = dev.get_feedback(MST_ID)
        start_pos = fb["q"] if fb else 0
        print(f"[INIT] Current position: {start_pos:.4f} rad")

        # ---- TEST 1: Pure damping (Kp=0, Kd=2) ----
        print("\n[TEST 1] Pure damping: Kp=0, Kd=2 (should NOT oscillate)")
        data = collect_data(dev, "hold_kp0_kd2", 3.0,
            lambda t: dev.control_mit(CAN_ID, kp=0, kd=2, q=0, dq=0, tau=0))
        results.append(analyze_oscillation(data, "hold_kp0_kd2"))

        # ---- TEST 2: Low Kp hold (Kp=3, Kd=2) ----
        print("[TEST 2] Low Kp hold: Kp=3, Kd=2")
        data = collect_data(dev, "hold_kp3_kd2", 3.0,
            lambda t: dev.control_mit(CAN_ID, kp=3, kd=2, q=start_pos, dq=0, tau=0))
        results.append(analyze_oscillation(data, "hold_kp3_kd2"))

        # ---- TEST 3: Medium Kp hold (Kp=10, Kd=3) ----
        print("[TEST 3] Medium Kp hold: Kp=10, Kd=3")
        data = collect_data(dev, "hold_kp10_kd3", 3.0,
            lambda t: dev.control_mit(CAN_ID, kp=10, kd=3, q=start_pos, dq=0, tau=0))
        results.append(analyze_oscillation(data, "hold_kp10_kd3"))

        # ---- TEST 4: High Kp hold (Kp=20, Kd=2) ----
        print("[TEST 4] High Kp hold: Kp=20, Kd=2")
        data = collect_data(dev, "hold_kp20_kd2", 3.0,
            lambda t: dev.control_mit(CAN_ID, kp=20, kd=2, q=start_pos, dq=0, tau=0))
        results.append(analyze_oscillation(data, "hold_kp20_kd2"))

        # ---- TEST 5: Step response (Kp=10, Kd=3, step +1 rad) ----
        print("[TEST 5] Step response: Kp=10, Kd=3, step +1 rad")
        target = start_pos + 1.0
        data = collect_data(dev, "step_kp10_kd3", 3.0,
            lambda t: dev.control_mit(CAN_ID, kp=10, kd=3, q=target, dq=0, tau=0))
        results.append(analyze_oscillation(data, "step_kp10_kd3"))

        # ---- TEST 6: Slow ramp (for friction measurement) ----
        print("[TEST 6] Slow ramp: constant tau=+0.3 Nm")
        data = collect_data(dev, "ramp_tau03", 3.0,
            lambda t: dev.control_mit(CAN_ID, kp=0, kd=0.5, q=0, dq=0, tau=0.3))
        results.append(analyze_oscillation(data, "ramp_tau03"))

        # ---- TEST 7: 200Hz send rate (like RhinoV2) ----
        print("[TEST 7] 200Hz hold: Kp=10, Kd=3 (RhinoV2 rate)")
        # First move back to start
        for _ in range(200):
            dev.control_mit(CAN_ID, kp=5, kd=3, q=start_pos, dq=0, tau=0)
            time.sleep(0.005)
        data = collect_data(dev, "hold_kp10_200hz", 3.0,
            lambda t: dev.control_mit(CAN_ID, kp=10, kd=3, q=start_pos, dq=0, tau=0),
            rate_hz=200)
        results.append(analyze_oscillation(data, "hold_kp10_200hz"))

        # Disable
        for _ in range(50):
            dev.control_mit(CAN_ID, kp=0, kd=2, q=0, dq=0, tau=0)
            time.sleep(0.01)
        for _ in range(5):
            dev.disable(CAN_ID)
            time.sleep(0.005)

        # Print report
        print_report(results)

        # Save results
        report_path = os.path.join(OUT_DIR, "report.json")
        with open(report_path, "w") as f:
            json.dump(results, f, indent=2)
        print(f"\nFull results saved to {report_path}")

    except Exception as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        import traceback; traceback.print_exc()
    finally:
        try:
            for _ in range(5):
                dev.disable(CAN_ID)
                time.sleep(0.005)
        except Exception:
            pass
        dev.close()
        print("[DONE]")


if __name__ == "__main__":
    main()
