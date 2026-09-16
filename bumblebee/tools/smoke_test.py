#!/usr/bin/env python3
"""Run smoke checks before starting simulation.

These checks catch structural errors such as a missing class definition
that syntax checks and --help can miss, without waiting for simulation.

Usage: python3 tools/smoke_test.py
Exit status is 0 for success and 1 for failure.
"""
import importlib.util
import math
import os
import subprocess
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
errors = []


def control(entry_name, condition_met, detail=""):
    print(f"  {'OK  ' if condition_met else 'ERROR'} {entry_name}" + (f" — {detail}" if detail and not condition_met else ""))
    if not condition_met:
        errors.append(entry_name)


def load_module(file_location, entry_name):
    spec = importlib.util.spec_from_file_location(entry_name, file_location)
    m = importlib.util.module_from_spec(spec)
    sys.modules[entry_name] = m
    spec.loader.exec_module(m)
    return m


def _schema(file_location, entry_name):
    import ast as _a
    t = _a.parse(open(file_location).read())
    h = r = pl = None
    for n in _a.walk(t):
        if isinstance(n, _a.Assign) and isinstance(n.value, _a.List):
            x = n.targets[0]
            if isinstance(x, _a.Name) and x.id == 'headers':
                h = len(n.value.elts)
            if isinstance(x, _a.Name) and x.id == 'row':
                r = len(n.value.elts)
        if isinstance(n, _a.AugAssign) and isinstance(n.target, _a.Name) and n.target.id == 'row':
            pl = len(n.value.elts)
    control(f"{entry_name}: header={h} line={r}+{pl}", h is not None and h == (r or 0) + (pl or 0),
            "a mismatch shifts the log columns")


print("1) is the guidance module actually loading (NOT syntax, runtime)")
try:
    g = load_module(os.path.join(ROOT, 'goat_cam_offset.py'), 'gco_smoke')
    control("module loaded", True)
    for cls, meths in (('FlightLogger', []), ('RedisListener', ['get_task']),
                       ('MavlinkManager', ['send_heading_target', 'send_altitude_target', 'run']),
                       ('AutopilotController', ['stabilize_pixel', '_log_state', 'run', 'clamp'])):
        var = hasattr(g, cls)
        control(f"class_type {cls}", var)
        if var:
            for me in meths:
                control(f"  {cls}.{me}", hasattr(getattr(g, cls), me))
    control("target data not leaked into code (no get_target_altitude)",
            not hasattr(getattr(g, 'RedisListener', object), 'get_target_altitude'))
except Exception as e:
    control("module loaded", False, repr(e))
    g = None

print("\n1b) teva.py (competition version: 1 Hz target 3D position) loading")
try:
    tv = load_module(os.path.join(ROOT, 'teva.py'), 'teva_smoke')
    control("teva module loaded", True)
    for cls in ('TargetTelemetry', 'MavlinkManager', 'AutopilotController'):
        control(f"teva.{cls}", hasattr(tv, cls))
    if hasattr(tv, 'TargetTelemetry'):
        for me in ('range_m', 'run'):
            control(f"  TargetTelemetry.{me}", hasattr(tv.TargetTelemetry, me))
except Exception as e:
    control("teva module loaded", False, repr(e))
    tv = None

def _check_area(file_location, entry_name, class_type='AutopilotController'):
    """Check that attributes read as self.X are initialized in __init__.

A missing assignment to self.aim_full_range_m once passed syntax,
--help and the earlier smoke check, then raised AttributeError during
simulation. This check detects that class of initialization error.
    """
    import ast as _a
    t = _a.parse(open(file_location).read())
    for n in _a.walk(t):
        if isinstance(n, _a.ClassDef) and n.name == class_type:
            assigned, read_value = set(), {}
            for x in _a.walk(n):
                if isinstance(x, _a.Attribute) and isinstance(x.value, _a.Name) and x.value.id == 'self':
                    if isinstance(x.ctx, _a.Store):
                        assigned.add(x.attr)
                    else:
                        read_value.setdefault(x.attr, x.lineno)
            methods = {f.name for f in n.body if isinstance(f, (_a.FunctionDef, _a.AsyncFunctionDef))}
            missing_fields = {k: v for k, v in read_value.items() if k not in assigned and k not in methods}
            control(f"{entry_name}: {class_type} attributes assigned", not missing_fields,
                    "read before assignment: " + ", ".join(f"{k} (row {v})" for k, v in sorted(missing_fields.items())))
            return
    control(f"{entry_name}: {class_type} found", False)


print("\n1c) Class fields: is each read self.X assigned?")
for _d in ('goat_cam_offset.py', 'teva.py'):
    try:
        _check_area(os.path.join(ROOT, _d), _d)
    except Exception as e:
        control(f"attribute initialization check ({_d})", False, repr(e))

print("\n2) CSV schema consistency (number of header fields equals row fields)")
for _file_name in ('goat_cam_offset.py', 'teva.py'):
    try:
        _schema(os.path.join(ROOT, _file_name), _file_name)
    except Exception as e:
        control(f"sky control ({_file_name})", False, repr(e))

print("\n3) Virtual gimbal mathematics (regression: pointing and centering)")
if g is not None:
    try:
        K = np.array([[4543, 0, 1025], [0, 4539, 569], [0, 0, 1]], float)
        Kinv = np.linalg.inv(K)

        def ry(d):
            r = math.radians(d)
            return np.array([[math.cos(r), 0, math.sin(r)], [0, 1, 0], [-math.sin(r), 0, math.cos(r)]])

        def raw_pixel(eps, th, roll=0.0):
            """RAW (x,y) pixels of the target in eps degrees above the horizon.             When roll != 0, the target also slides horizontally; It is wrong to assume x is constant."""
            e = math.radians(eps)
            dvec = np.array([math.cos(e), 0.0, -math.sin(e)])
            R = g.compute_R_b_e(math.radians(roll), math.radians(th), 0.0)
            h = K @ (g.R_c_b_T @ (R.T @ dvec))
            return h[0] / h[2], h[1] / h[2]

        def exey(eps, th, aim, roll=0.0):
            x, y = raw_pixel(eps, th, roll)
            R = g.compute_R_b_e(math.radians(roll), math.radians(th), 0.0)
            rb = g.R_c_b @ (Kinv @ np.array([x, y, 1.0]))
            hh = K @ (g.R_c_b_T @ (ry(aim) @ (R @ rb)))
            sx, sy = hh[0] / hh[2], hh[1] / hh[2]
            return (math.degrees(math.atan((sx - 1025) / 4543)),
                    math.degrees(math.atan((sy - 569) / 4539)))

        def ey(eps, th, aim, roll=0.0):
            return exey(eps, th, aim, roll)[1]

        # (a) de-rotation: equal-altitude target at center every roll/pitch at am=0
        bad_samples = [(rl, th) for rl in (0, 10, 20, 30) for th in (-2.77, 0.0, 4.19)
                if max(abs(v) for v in exey(0.0, th, 0.0, rl)) > 1e-3]
        control("derotation removes roll and pitch", not bad_samples, f"deviations: {bad_samples[:3]}")
        # (b) equilibrium relation: ey=0 <=> eps = -aim
        control("equilibrium eps = -aim", all(abs(ey(-a, -2.77, a)) < 1e-3 for a in (-6, -2.8, 0, 2.8, 6)))
        # (c) aim = -theta places the target at the RAW frame center
        for th in (-2.77, 4.19):
            _, yv = raw_pixel(th, th)   # equilibrium eps = theta
            control(f"aim=-theta (theta={th:+.2f}) -> raw pixel centered",
                    abs(yv - 569) < 2.0, f"y={yv:.1f}")
    except Exception as e:
        control("gimbal mathematics", False, repr(e))

print("\n4) Can the helper tools be opened with --help?")
for t_ in ('pid_plot.py', 'ground_truth_logger.py', 'test_setup.py'):
    p = os.path.join(ROOT, 'tools', t_)
    if not os.path.exists(p):
        control(t_, False, "no file"); continue
    rc = subprocess.run([sys.executable, p, '--help'], capture_output=True, timeout=60).returncode
    control(t_, rc == 0, f"exit code {rc}")

print("\n" + ("SMOKE TEST CLEAN — sim can be entered" if not errors
              else f"SMOKE TEST FAIL ({len(errors)}): " + ", ".join(errors)))
sys.exit(1 if errors else 0)
