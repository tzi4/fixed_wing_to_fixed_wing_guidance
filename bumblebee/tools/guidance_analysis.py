#!/usr/bin/env python3
"""
Reconstruct visual-guidance output from the aircraft's own .BIN log.

This works without the guidance CSV. ArduPilot records the
MAV_CMD_GUIDED_CHANGE_ALTITUDE (43001) and _CHANGE_HEADING (43002)
commands sent by goat_gimbal_aircraft.py in MAVC messages.

Reconstruction chain (Ki_alt = 0, so there is no integral term):

    MAVC[43001].Z          = target_alt          (commanded absolute altitude)
    POS.RelHomeAlt         = current_alt         (altitude read by guidance)
    cmd_alt_m              = target_alt - current_alt        <- PID output
    p_y                    = -cmd_alt_m / Kp_alt             <- angle error after deadzone
    stab_error_y_deg       ≈ p_y + sign(p_y)*deadzone        <- virtual-gimbal output

    MAVC[43002].P2         = target_heading
    cmd_head_deg           = wrap180(target_heading - yaw)
    p_x                    = cmd_head_deg / Kp_heading
    stab_error_x_deg       ≈ p_x + sign(p_x)*deadzone

Usage: python3 tools/guidance_analysis.py <log.BIN> [...]
"""
import sys, os, math
import numpy as np
from pymavlink import mavutil

KP_ALT, KP_HDG, DEADZONE = 2.5, 3.0, 0.4
CLAMP_LO, CLAMP_HI = -5.0, 2.0      # goat_gimbal_aircraft.py latest version
FY = 4510.0                          # real camera calibration

def wrap180(a): return (a + 180.0) % 360.0 - 180.0

def analyze(path):
    m = mavutil.mavlink_connection(path)
    alt_cmd=[]; hdg_cmd=[]; pos=[]; att=[]; gps=[]; mode=[]
    while True:
        x = m.recv_match(type=['MAVC','POS','ATT','GPS','MODE'])
        if x is None: break
        t = x.get_type()
        if t=='MAVC':
            if x.Cmd==43001: alt_cmd.append((x.TimeUS/1e6, x.Z, x.P3))
            elif x.Cmd==43002: hdg_cmd.append((x.TimeUS/1e6, x.P2, x.P3))
        elif t=='POS': pos.append((x.TimeUS/1e6, x.RelHomeAlt))
        elif t=='ATT': att.append((x.TimeUS/1e6, x.Pitch, x.Roll, x.Yaw))
        elif t=='GPS': gps.append((x.TimeUS/1e6, x.Spd))
        elif t=='MODE': mode.append((x.TimeUS/1e6, x.Mode))

    print(f"\n{'='*70}\n{os.path.basename(path)}")
    if not alt_cmd:
        print("  There is no guidance command (video guidance did not work in this log).")
        return

    ac=np.array(alt_cmd); hc=np.array(hdg_cmd); po=np.array(pos); at=np.array(att); gp=np.array(gps)

    # --- GUIDED time and LOCK RATE ---
    guided_s = 0.0
    if mode:
        tm=[q[0] for q in mode]+[at[-1,0]]
        for i,(t0,md) in enumerate(mode):
            if int(md)==15: guided_s += tm[i+1]-t0
    span = ac[-1,0]-ac[0,0]
    print(f"  GUIDED duration: {guided_s:.0f} s | guidance command: {len(ac)} quantity | "
          f"command range: {span:.0f} s")
    # the code is sending one at 0.1 s -> theoretical max 10 Hz; realized rate = lock time
    lock_s = len(ac)*0.1
    print(f"  >>> LOCK DURATION ≈ {lock_s:.0f} s  "
          f"({100*lock_s/max(guided_s,1):.1f}% of GUIDED)")
    # seamless lock blocks
    dt_cmd=np.diff(ac[:,0])
    blocks=np.split(np.arange(len(ac)), np.where(dt_cmd>0.5)[0]+1)
    bl=sorted((len(b)*0.1 for b in blocks), reverse=True)
    print(f"  number of lock blocks: {len(bl)} | longest: {bl[0]:.1f} s | "
          f"median: {np.median(bl):.1f} s | >5 s: {sum(1 for x in bl if x>5)}")

    # --- param3 (climb speed limit) ---
    p3=np.unique(ac[:,2])
    print(f"  MAV_CMD_GUIDED_CHANGE_ALTITUDE param3 (climb speed): {p3}"
          f"  {'-> NO LIMIT' if np.allclose(p3,0) else ''}")
    if len(hc): print(f"  CHANGE_HEADING param3 (rotation speed): "
                      f"{np.min(hc[:,2]):.2f} .. {np.max(hc[:,2]):.2f} °/s")

    # --- Decode output PID ---
    cur = np.interp(ac[:,0], po[:,0], po[:,1])
    cmd = ac[:,1] - cur
    ok = np.abs(cmd) < 60          # shoot ground safety (target_alt<10 -> 10) pellets
    cmd = cmd[ok]; tt = ac[ok,0]
    print(f"\n --- VERTICAL AXIS ---")
    print(f"  cmd_alt_m: median {np.median(cmd):+.2f} m | mean {cmd.mean():+.2f} | "
          f"std {cmd.std():.2f} | p5 {np.percentile(cmd,5):+.2f} | p95 {np.percentile(cmd,95):+.2f}")
    sat_lo = float((cmd <= CLAMP_LO+0.05).mean()*100)
    sat_hi = float((cmd >= CLAMP_HI-0.05).mean()*100)
    outside = float(((cmd < CLAMP_LO-0.1)|(cmd > CLAMP_HI+0.1)).mean()*100)
    print(f"  clamp(-5,+2) saturation: %{sat_lo:.1f} at the lower limit, %{sat_hi:.1f} at the upper limit")
    print(f"  EXCLUDING clamp: %{outside:.1f}  "
          f"-> {'NO CLAMP (in this version)' if outside>5 else 'clamp was active'}")

    # convert to angle error
    p_y = -cmd/KP_ALT
    stab_y = p_y + np.sign(p_y)*DEADZONE
    print(f"  back-solved stab_error_y_deg: median {np.median(stab_y):+.2f}° | "
          f"mean {stab_y.mean():+.2f}° | std {stab_y.std():.2f}°")
    print(f"  NOTE: this is a CLOSED LOOP residual error, NOT (b+eps). plant one")
    print(f"       Since it is an integrator, P-control drives the permanent error to zero;")
    print(f"       (b+eps) deviation is not in the error signal, but in the AIRCRAFT'S RELATIVE TO TARGET")
    print(f"       appears in POSITION. (b+eps) is measured only by raw pixel yield.")

    # --- HORIZONTAL AXIS (comparison) ---
    if len(hc):
        yaw = np.interp(hc[:,0], at[:,0], at[:,3])
        ch = np.array([wrap180(a-b) for a,b in zip(hc[:,1], yaw)])
        ok2 = np.abs(ch) < 40
        ch = ch[ok2]
        p_x = ch/KP_HDG
        stab_x = p_x + np.sign(p_x)*DEADZONE
        print(f"\n  --- HORIZONTAL AXIS ---")
        print(f"  cmd_head_deg: median {np.median(ch):+.2f}° | std {ch.std():.2f}")
        print(f"  back-solved stab_error_x_deg: median {np.median(stab_x):+.2f}° | "
              f"std {stab_x.std():.2f}°")
        print(f"  (same closed-loop warning applies)")

    # --- DE-ROTATION TEST: Does stab_error_y correlate with pitch? ---
    pit = np.interp(tt, at[:,0], at[:,1])
    if len(tt) > 200 and pit.std() > 0.5:
        # discard the slow component (actual movement of the target), look at the fast oscillation
        w = 31
        k = np.ones(w)/w
        hp = lambda v: v - np.convolve(v, k, mode='same')
        a_, b_ = hp(stab_y), hp(pit)
        if b_.std() > 1e-6:
            sl = np.polyfit(b_, a_, 1)[0]; r = np.corrcoef(b_, a_)[0,1]
            print(f"\n --- DE-ROTATION TEST ---")
            print(f"  d(stab_error_y)/d(pitch) = {sl:+.3f} (EXPECTED 0.000) | r = {r:+.3f}")
            print(f"  WARNING: This test is CONFIDENT in closed loop. Error -> altitude command")
            print(f"  -> pitch causality already produces negative correlation. Clean")
            print(f"  raw pixel required for rigidity testing: tools/camera_angle_test.py")

    # --- does the deviation drift over time ---
    q = np.array_split(np.arange(len(stab_y)), 4)
    meds = [float(np.median(stab_y[i])) for i in q if len(i)>20]
    if len(meds)>1:
        print(f"  quartile medians: " + " ".join(f"{v:+.2f}°" for v in meds)
              + f"  (spread {max(meds)-min(meds):.2f}°)")

if __name__ == '__main__':
    if len(sys.argv) < 2: print(__doc__); sys.exit(1)
    for p in sys.argv[1:]: analyze(p)
