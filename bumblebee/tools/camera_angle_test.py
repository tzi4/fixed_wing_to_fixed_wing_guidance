#!/usr/bin/env python3
"""
Check the camera-to-autopilot angle and virtual-gimbal operation.

This requires a guidance CSV because the aircraft .BIN log contains no
camera measurements. Use a flight_log_guided_*.csv supplied by Emir.

Usage:
    python3 tools/camera_angle_test.py flight_log_guided_YYYYMMDD_HHMMSS.csv

Derived for stabilize_pixel in goat_gimbal_aircraft.py:
    raw_error_y_deg  = -(phi - theta + eps)
    stab_error_y_deg = -(phi + b + eps)
Here phi is target elevation in the ground frame, theta is actual
body pitch, b is reported minus actual autopilot pitch, and eps is
camera mounting pitch relative to the body, positive downward.

Three independent checks:
  1. Rigidity: d(raw_error_y_deg)/d(pitch_deg) should equal +1.000.
     A deviation indicates camera movement relative to the body or
     an incorrect fy. Rigid mounting is essential for stabilization.
  2. Derotation: stab_error_y_deg should be uncorrelated with pitch_deg.
     Correlation indicates incomplete removal of pitch motion.
  3. Constant offset: mean stab_error_y_deg = -(phi + b + eps).
     With mean target elevation near the horizon, phi is approximately
     zero and this gives the negative combined b + eps offset. The two
     offsets cannot be identified separately from these measurements.
     One correction value can compensate for both.

The variance ratio var(stab)/var(raw) should be much smaller than one
when the virtual gimbal is working.
"""
import sys, csv, math
import numpy as np

def load(path):
    rows = list(csv.DictReader(open(path)))
    def col(name):
        out = []
        for r in rows:
            try: out.append(float(r[name]))
            except (ValueError, KeyError, TypeError): out.append(np.nan)
        return np.array(out)
    d = {k: col(k) for k in ['elapsed_s','pitch_deg','roll_deg','yaw_deg',
                             'raw_error_y_deg','stab_error_y_deg',
                             'raw_error_x_deg','stab_error_x_deg',
                             'bbox_center_y','stab_y','current_alt_m']}
    d['target_found'] = np.array([r.get('target_found','0') for r in rows])
    d['mode'] = np.array([r.get('flight_mode','') for r in rows])
    return d

def target_present(d):
    """Only lines where the target is actually seen and the numbers make sense."""
    m = (d['target_found'] == '1')
    for k in ['pitch_deg','raw_error_y_deg','stab_error_y_deg']:
        m &= np.isfinite(d[k])
    m &= np.abs(d['raw_error_y_deg']) < 20
    m &= np.abs(d['stab_error_y_deg']) < 20
    return m

def hp(x, w):
    """High pass: throw the slow motion of the target, drop the pitch oscillation."""
    k = np.ones(w)/w
    return x - np.convolve(x, k, mode='same')

def main(path):
    d = load(path)
    m = target_present(d)
    n = int(m.sum())
    print(f"File: {path}")
    print(f"Number of target samples: {n} / {len(d['elapsed_s'])}")
    if n < 300:
        print("INSUFFICIENT DATA (>=300 required). Testing cannot be done."); return

    t   = d['elapsed_s'][m]
    pit = d['pitch_deg'][m]
    role_name = d['roll_deg'][m]
    raw = d['raw_error_y_deg'][m]
    stb = d['stab_error_y_deg'][m]
    rawx= d['raw_error_x_deg'][m]
    stbx= d['stab_error_x_deg'][m]

    dt = np.median(np.diff(t)) if len(t) > 1 else 0.033
    W = max(5, int(round(3.0/dt)))     # 3 sec high pass window

    print(f"pitch range: {pit.min():.1f}° .. {pit.max():.1f}° (std {pit.std():.2f}°)")
    if pit.std() < 0.8:
        print("WARNING: pitch is almost unchanged; TEST 1/2 would be meaningless.")

    # ---- TEST 1: RIGIDITY ----
    print("\n=== TEST 1 — RIGIDITY (is the camera fixed to the body?) ===")
    a = hp(raw, W); b_ = hp(pit, W)
    if b_.std() > 1e-6:
        slope, icept = np.polyfit(b_, a, 1)
        r = np.corrcoef(b_, a)[0,1]
        print(f"  d(raw_error_y)/d(pitch) = {slope:+.3f} (EXPECTED: +1.000)")
        print(f"  correlation r = {r:+.3f} (|r| > 0.7 so that the slope is significant)")
        if abs(r) < 0.7:
            print("  -> Correlation is poor: target is too moving or there is no pitch oscillation. The result is unreliable.")
        elif abs(slope - 1.0) < 0.12:
            print("  -> PASSED. The camera is rigidly attached to the body, the scale is correct.")
        elif slope < 0.88:
            print(f"  -> FAIL. Pitch is less than 1: camera rotates less than the measured aircraft pitch")
            print(f"     (the mount is stretching) or fy is actually smaller than the {slope:.3f}x.")
        else:
            print(f"  -> FAIL. Slope greater than 1: fy may have been entered too small.")

    # ---- TEST 2: DEROTATION ----
    print("\n=== TEST 2 — DE-ROTATION (does the gimbal clear the pitch?) ===")
    c = hp(stb, W)
    if b_.std() > 1e-6:
        slope2, _ = np.polyfit(b_, c, 1)
        r2 = np.corrcoef(b_, c)[0,1]
        print(f"  d(stab_error_y)/d(pitch) = {slope2:+.3f} (EXPECTED: 0.000)")
        print(f"  correlation r = {r2:+.3f} (EXPECTED: ~0)")
        if abs(slope2) < 0.15: print("  -> PASSED. Pitch has been cleared.")
        else: print(f"  -> FAIL. {abs(slope2)*100:.0f}% of the pitch leaks into the stabilized output.")

    vr = np.var(c)/np.var(a) if np.var(a) > 0 else float('nan')
    print(f"  variance ratio var(stab)/var(raw) = {vr:.3f} (EXPECTED: <0.25)")
    if vr < 0.25: print("  -> The virtual gimbal works on the vertical axis.")
    elif vr < 1.0: print("  -> Partially working.")
    else: print("  -> IT DOES NOT WORK (in fact, it makes it worse) IF the gimbal is on the vertical axis.")

    # ---- TEST 3: FIXED OFFSET ----
    print("\n=== TEST 3 — CONSTANT OFFSET (b + eps) ===")
    med = np.median(stb); mean = np.mean(stb)
    print(f"  stab_error_y_deg: median {med:+.2f}°  mean {mean:+.2f}°  std {stb.std():.2f}°")
    print(f"  -> total vertical deviation (b + eps) ≈ {-med:+.2f}° (if target is at mean horizon level)")
    print(f"  -> goat_gimbal_aircraft.py Altitude command deviation with Kp_alt=2.5: {abs(med)*2.5:.1f} m")
    print(f"     (clamp range -5..+2 m — {'SATURATES' if abs(med)*2.5 > 2 else 'within limits'})")
    print(f"  -> equivalent pixel shift (fy=4510): {abs(med)*math.pi/180*4510:.0f} px")

    # ---- Horizontal axis (for comparison) ----
    print("\n=== HORIZONTAL AXIS (comparison) ===")
    ax = hp(rawx, W); cx = hp(stbx, W); br = hp(role_name, W)
    vrx = np.var(cx)/np.var(ax) if np.var(ax) > 0 else float('nan')
    print(f"  variance ratio var(stab_x)/var(raw_x) = {vrx:.3f}")
    print(f"  stab_error_x_deg median {np.median(stbx):+.2f}°")
    if br.std() > 1e-6:
        print(f"  d(stab_error_x)/d(roll) = {np.polyfit(br, cx, 1)[0]:+.3f} (EXPECTED: 0.000)")

    # ---- Is the offset shifting? ----
    print("\n=== IS THE OFFSET CONSTANT OVER TIME? ===")
    q = np.array_split(np.arange(len(stb)), 4)
    meds = [float(np.median(stb[i])) for i in q if len(i) > 30]
    print("  quartile medians: " + "  ".join(f"{v:+.2f}°" for v in meds))
    if len(meds) > 1:
        rng = max(meds) - min(meds)
        print(f"  spread {rng:.2f}°")
        print("  -> " + ("STILL. The static calibration issue is fixed with an odd number."
                         if rng < 1.0 else
                         "IT'S SLIDING. The mount flexes or the pitch error changes with the flight condition."))

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    for p in sys.argv[1:]:
        main(p); print("\n" + "="*66 + "\n")
