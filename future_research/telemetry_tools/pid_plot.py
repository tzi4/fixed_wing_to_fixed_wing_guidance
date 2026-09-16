#!/usr/bin/env python3
"""
Plot PID logs and quantify controller behavior.

Read CSV output from goat_cam_offset.py to measure axis errors, command
strength, saturation and oscillation periods. Supplying independent
ground truth from tools/ground_truth_logger.py also reveals whether the
controller oscillates while the target flies straight.

Usage:
    python3 tools/pid_plot.py flight_logs/flight_log_guided_*.csv
    python3 tools/pid_plot.py guidance.csv --ground-truth flight_logs/ground_truth_*.csv
    python3 tools/pid_plot.py guidance.csv --outdir reports/pid_2026-07-29
"""
import argparse
import csv
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# --- Reference baselines (measured, not fitted) --- Actual flight (july_1_logs 79/80): cmd_heading std 12.3-12.8 deg, period 4.5-6.0 s.  GOOD configuration on sim (Erenimbus A/B, 2026-07-28): cmd std 0.50, |error| 0.44, no oscillation.  BAD configuration in sim (bumblebee): cmd std 18.65, |error| 5.74, period 1.99 s.
REF = {
    'flight_command_std': (12.3, 12.8),
    'flight_period': (4.5, 6.0),
    'good_abs_error': 0.44,
    'bad_abs_error': 5.74,
    'bad_period': 1.99,
}

# OSCILLATION DETECTION GATES
#A false alarm occurred with mean vertical |error| = 0.30 degrees,
#94.2% of samples inside the deadzone, command standard deviation 0.25 m
#and saturation 0.1%. The ungated report gave a dominant period of
#1.25 s and 178.4 sign changes per minute. Those were measurement-noise
#zero crossings. The FFT peak accounted for only 6.7% of total power.
#To avoid such false alarms, require both gates before reporting
#period or sign-change metrics as evidence of oscillation.
#
#  1. Amplitude gate: standard deviation of the mean-subtracted error.
#     For a sinusoid, peak = std * sqrt(2), so std >= deadzone implies
#     peak >= 1.41 * deadzone. Compute std after clipping at the 1st and
#     99th percentiles to reduce sensitivity to isolated detection spikes.
#     With a 0.4-degree deadzone, clean vertical/horizontal values 0.19/0.08
#     fail the gate. T1A vertical 1.08 and Bumblebee horizontal/vertical
#     values 5.56/2.05 pass.
#  2. Spectral dominance gate: the FFT peak and its +/-2 neighboring bins
#     must contain at least 20% of total power. White noise is near 1%.
#     Measured oscillations yielded 26%-75%: T1A vertical 74.6%, Bumblebee
#     horizontal 30.2%, and vertical 26.5%. The false alarm yielded 6.7%.
#     The 20% threshold separates these cases and exceeds the noise floor
#     by approximately an order of magnitude.
#
#Count sign changes with hysteresis. A reversal must cross the band in
#the new direction, using band = max(deadzone, amplitude/2). Ignore zero
#crossings that stay within the band. Retain ungated raw metrics in the
#report, marked as having no diagnostic value.
AMPLITUDE_THRESHOLD_FACTOR = 1.0      # amplitude threshold = factor * deadzone
DOMINANT_THRESHOLD_PCT = 20.0     # fraction of total power in the peak +/-2 bins [%]
MIN_TRANSITIONS = 3              # Minimum out-of-band direction change required to claim period


def read_csv(path):
    with open(path, newline='') as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        sys.exit(f"ERROR: {path} is empty.")
    cols = {}
    for k in rows[0]:
        vals = []
        for r in rows:
            v = (r.get(k) or '').strip()
            try:
                vals.append(float(v))
            except ValueError:
                vals.append(np.nan)
        cols[k] = np.array(vals)
    cols['_raw'] = rows
    return cols


def col(d, name, default=None):
    v = d.get(name)
    if v is None or np.all(np.isnan(v)):
        return default
    return v


def _trimmed_std(xc):
    """1%-99% trimmed std — do not inflate single frame detection bounce amplitude."""
    if len(xc) < 5:
        return float(np.std(xc))
    lo, hi = np.percentile(xc, [1.0, 99.0])
    return float(np.std(np.clip(xc, lo, hi)))


def _hysteresis_transitions(t, x, band):
    """Return times when the signal reverses direction beyond the hysteresis band.

Simply counting zero crossings misclassifies noise as oscillation,
as in the clean case reporting 178 crossings per minute. Count a
reversal only after crossing the band in the new direction.
    """
    ts = []
    state_value = 0
    for i in range(len(x)):
        v = x[i]
        if v > band and state_value <= 0:
            if state_value != 0:
                ts.append(t[i])
            state_value = 1
        elif v < -band and state_value >= 0:
            if state_value != 0:
                ts.append(t[i])
            state_value = -1
    return np.array(ts)


def measure_oscillation(t, x, deadzone):
    """Measure oscillation metrics and evaluate the detection gates.

When oscillation_detected is False, period and sign-change metrics do not
establish oscillation. short_reason and reason identify the failed gate.
    """
    s = {
        'amplitude': None, 'amplitude_threshold': AMPLITUDE_THRESHOLD_FACTOR * deadzone,
        'amplitude_passed': False, 'dominance_passed': False, 'oscillation_detected': False,
        'zero_crossing_period': None, 'fft_period': None, 'peak_power_pct': None,
        'transition_count': 0, 'sign_changes_per_min': None, 'band': None,
        'raw_zero_crossing_period': None, 'raw_sign_changes_per_min': None,
        'duration_s': 0.0, 'reason': 'measurement could not be made',
        'short_reason': 'measurement could not be made',
    }
    good = ~np.isnan(x)
    t, x = t[good], x[good]
    if len(x) < 20 or len(t) < 2:
        s['reason'] = s['short_reason'] = 'measurement could not be made (sample < 20)'
        return s
    dur = float(t[-1] - t[0])
    s['duration_s'] = dur
    xc = x - np.mean(x)
    amplitude = _trimmed_std(xc)
    s['amplitude'] = amplitude
    s['amplitude_passed'] = amplitude >= s['amplitude_threshold']
    if amplitude < 1e-9:
        s['reason'] = s['short_reason'] = 'no oscillation (signal stable)'
        return s

    # Moving average of ~0.3 s: frame by frame measurement noise does not count as direction change
    dt_med = float(np.median(np.diff(t)))
    w = max(3, int(round(0.3 / dt_med))) if dt_med > 0 else 3
    if w % 2 == 0:
        w += 1
    if len(xc) > w:
        kern = np.ones(w) / w
        xs = np.convolve(xc, kern, mode='same')
        xs[:w] = xc[:w]; xs[-w:] = xc[-w:]
    else:
        xs = xc

    # RAW (ungated) numbers — old behavior. It is indelible, but has no diagnostic value.
    sign_value = np.sign(xs)
    raw_idx = np.where(np.diff(sign_value) != 0)[0]
    if len(raw_idx) >= 3:
        s['raw_zero_crossing_period'] = 2.0 * float(np.median(np.diff(t[raw_idx])))
    if dur > 0:
        s['raw_sign_changes_per_min'] = len(raw_idx) / dur * 60.0

    # HYSTERESIS: transitions that do not leave the noise band are not counted
    band = max(deadzone, 0.5 * amplitude)
    s['band'] = band
    ts = _hysteresis_transitions(t, xs, band)
    s['transition_count'] = int(len(ts))
    if dur > 0:
        s['sign_changes_per_min'] = len(ts) / dur * 60.0
    if len(ts) >= MIN_TRANSITIONS:
        # Between successive out-of-band direction changes = half period
        s['zero_crossing_period'] = 2.0 * float(np.median(np.diff(ts)))

    # FFT (resampled to equal spacing)
    if len(t) > 32 and dur > 1e-6:
        n = min(4096, max(64, len(t)))
        ti = np.linspace(t[0], t[-1], n)
        xi = np.interp(ti, t, xc)
        dt = ti[1] - ti[0]
        sp = np.abs(np.fft.rfft(xi * np.hanning(n))) ** 2
        fr = np.fft.rfftfreq(n, dt)
        m = fr > 0.05  # Address long trends from 20 s
        if np.any(m) and np.sum(sp[m]) > 0:
            k = int(np.argmax(sp[m]))
            f0 = fr[m][k]
            # fraction of total power in the peak +/-2 bins
            spm = sp[m]
            lo, hi = max(0, k - 2), min(len(spm), k + 3)
            s['peak_power_pct'] = float(100.0 * np.sum(spm[lo:hi]) / np.sum(spm))
            if f0 > 0:
                s['fft_period'] = 1.0 / f0
    s['dominance_passed'] = (s['peak_power_pct'] is not None
                         and s['peak_power_pct'] >= DOMINANT_THRESHOLD_PCT)

    # ---gate DECISION: both conditions are judged and reported separately ---
    if not s['amplitude_passed']:
        s['short_reason'] = 'no oscillation (under amplitude deadzone)'
        s['reason'] = ('amplitude gate REQUIRED: trimmed std val %.2f < threshold val %.2f '
                        '(= deadzone). The signal does not leave the measurement noise band.'
                        % (amplitude, s['amplitude_threshold']))
    elif s['peak_power_pct'] is None:
        s['short_reason'] = 'no dominant frequency (FFT could not be done)'
        s['reason'] = 'dominance gate FAIL: record is too short for FFT.'
    elif not s['dominance_passed']:
        s['short_reason'] = ('no dominant frequency (peak power %%%.1f)' % s['peak_power_pct'])
        s['reason'] = ('dominance gate FAILED: FFT top (+-2k) total power '
                        'It carries only %%%.1f\', the threshold is %%%.0f. broadband '
                        'noise, not single frequency oscillation.'
                        % (s['peak_power_pct'], DOMINANT_THRESHOLD_PCT))
    elif len(ts) < MIN_TRANSITIONS:
        s['short_reason'] = 'no oscillation (insufficient out-of-band direction change)'
        s['reason'] = ('Out-of-band direction changes %d < %d: signal does not repeatedly leave '
                        'the +-%.2f band.' % (len(ts), MIN_TRANSITIONS, band))
    else:
        s['oscillation_detected'] = True
        s['short_reason'] = 'oscillation YES'
        s['reason'] = ('PASS: amplitude %.2f >= threshold %.2f AND peak power %%%.1f >= %%%.0f AND '
                        '%d out-of-band direction change.'
                        % (amplitude, s['amplitude_threshold'], s['peak_power_pct'],
                           DOMINANT_THRESHOLD_PCT, len(ts)))
    return s


def axis_stats(t, err, cmd_raw, cmd, sat, deadzone=0.4):
    s = {}
    good = ~np.isnan(err)
    e = err[good]
    s['n'] = int(len(e))
    if len(e) == 0:
        return s
    s['mean_abs_error'] = float(np.mean(np.abs(e)))
    s['error_std'] = float(np.std(e))
    s['error_rms'] = float(np.sqrt(np.mean(e ** 2)))
    s['max_abs_error'] = float(np.max(np.abs(e)))
    s['within_deadzone_pct'] = float(100.0 * np.mean(np.abs(e) < deadzone))
    # Gate oscillation metrics. A failed gate marks period and sign-change values as nondiagnostic.
    s.update(measure_oscillation(t, err, deadzone))
    dur = float(t[good][-1] - t[good][0]) if len(t[good]) > 1 else 0.0
    s['duration_s'] = dur
    if cmd is not None:
        c = cmd[~np.isnan(cmd)]
        if len(c):
            s['cmd_std'] = float(np.std(c))
            s['mean_abs_command'] = float(np.mean(np.abs(c)))
    if sat is not None:
        sv = sat[~np.isnan(sat)]
        if len(sv):
            s['saturation_pct'] = float(100.0 * np.mean(sv > 0.5))
    return s


def fmt(v, n=2, suffix=''):
    return '—' if v is None else f"{v:.{n}f}{suffix}"


def axis_verdict(entry_name, s, measurement_unit):
    """Flat assessment for single axis.

    The oscillation assertion is made ONLY if the gate has been passed. Small error + high deadzone occupancy + low saturation + gate remaining = HEALTHY; not ambiguous.
    """
    error = s.get('mean_abs_error')
    dz = s.get('within_deadzone_pct')
    sat = s.get('saturation_pct')
    per = s.get('fft_period') if s.get('fft_period') is not None else s.get('zero_crossing_period')
    issue = []
    if s.get('oscillation_detected'):
        if per is not None and per <= 3.0:
            issue.append('**LIMIT CYCLE**: detected oscillation with a %.2f s period '
                         '(amplitude %.2f deg, peak power %%%.1f, %.1f out-of-band reversals per minute '
                         'reversals); bad configuration reference ~%.2f s'
                         % (per, s.get('amplitude') or float('nan'),
                            s.get('peak_power_pct') or float('nan'),
                            s.get('sign_changes_per_min') or float('nan'), REF['bad_period']))
        else:
            issue.append('slow oscillation exists: period %s s, amplitude %.2f deg, peak power %%%.1f '
                         '(not in the limit-cycle band)'
                         % (fmt(per), s.get('amplitude') or float('nan'),
                            s.get('peak_power_pct') or float('nan')))
    if error is not None and error > 2.0 * REF['good_abs_error']:
        issue.append('mean |error| %.2f deg; good configuration %.2f deg (bad configuration %.2f deg)'
                     % (error, REF['good_abs_error'], REF['bad_abs_error']))
    if sat is not None and sat > 10.0:
        issue.append("command reaches its clamp for %.1f%% of the time" % sat)
    if dz is not None and dz < 50.0:
        issue.append('Duration in deadzone is only %%%.1f' % dz)
    if not issue:
        return ('- **%s: HEALTHY.** mean |error| %s deg, deadzone occupancy %s%%, '
                'command std %s %s, saturation %s%%. %s'
                % (entry_name, fmt(error), fmt(dz, 1), fmt(s.get('cmd_std')), measurement_unit, fmt(sat, 1),
                   s.get('reason', '')))
    return '- **%s: ISSUES DETECTED.** %s.' % (entry_name, '; '.join(issue))


def plot_axis(outdir, tag, title, t, err, terms, cmd_raw, cmd, extra, deadzone):
    n = 3 + (1 if extra else 0)
    fig, ax = plt.subplots(n, 1, figsize=(13, 3.0 * n), sharex=True)
    fig.suptitle(title, fontsize=13, fontweight='bold')

    ax[0].plot(t, err, lw=0.9, color='#c0392b', label='error')
    ax[0].axhspan(-deadzone, deadzone, color='#95a5a6', alpha=0.25, label=f'deadzone ±{deadzone}')
    ax[0].axhline(0, color='k', lw=0.6)
    ax[0].set_ylabel('error [value]'); ax[0].legend(loc='upper right', fontsize=8)
    ax[0].grid(alpha=0.3)

    for name, series, c in terms:
        if series is not None:
            ax[1].plot(t, series, lw=0.9, label=name, color=c)
    ax[1].axhline(0, color='k', lw=0.6)
    ax[1].set_ylabel('PID terms'); ax[1].legend(loc='upper right', fontsize=8)
    ax[1].grid(alpha=0.3)

    if cmd_raw is not None:
        ax[2].plot(t, cmd_raw, lw=0.8, color='#7f8c8d', label='raw command (pre-clamp)')
    if cmd is not None:
        ax[2].plot(t, cmd, lw=1.1, color='#2980b9', label='executed command')
    ax[2].axhline(0, color='k', lw=0.6)
    ax[2].set_ylabel('command'); ax[2].legend(loc='upper right', fontsize=8)
    ax[2].grid(alpha=0.3)

    if extra:
        for name, series, c in extra:
            if series is not None:
                ax[3].plot(t, series, lw=0.9, label=name, color=c)
        ax[3].set_ylabel('extra_text'); ax[3].legend(loc='upper right', fontsize=8)
        ax[3].grid(alpha=0.3)

    ax[-1].set_xlabel('elapsed duration [s]')
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    p = os.path.join(outdir, f'{tag}.png')
    fig.savefig(p, dpi=110)
    plt.close(fig)
    return p


def main():
    ap = argparse.ArgumentParser(description='guidance CSV -> PID graphics + honest assessment')
    ap.add_argument('csv', help='goat_cam_offset.py flight_log_guided_*.csv')
    ap.add_argument('--ground-truth', default=None, help='tools/ground_truth_logger.py CSV (ground truth)')
    ap.add_argument('--outdir', default=None, help='output directory')
    ap.add_argument('--deadzone', type=float, default=0.4)
    args = ap.parse_args()

    d = read_csv(args.csv)
    outdir = args.outdir or os.path.join('reports', 'pid_' + os.path.basename(args.csv).replace('.csv', ''))
    os.makedirs(outdir, exist_ok=True)

    t = col(d, 'elapsed_s')
    if t is None:
        sys.exit('ERROR: elapsed_s does not have a shot.')
    state = [r.get('system_state', '') for r in d['_raw']]
    tracking = np.array([s == 'TRACKING' for s in state])
    ex = col(d, 'stab_error_x_deg'); ey = col(d, 'stab_error_y_deg')
    if ex is None:
        sys.exit('ERROR: No stab_error_x_deg — is this a guidance CSV\'si?')

    figs = []
    # --- HORIZONTAL AXIS (heading) ---
    figs.append(plot_axis(
        outdir, 'horizontal_axis', 'HORIZONTAL AXIS (ex -> heading)', t, ex,
        [('P', col(d, 'p_term_heading'), '#27ae60'),
         ('I', col(d, 'i_term_heading'), '#f39c12'),
         ('D', col(d, 'd_term_heading'), '#8e44ad')],
        col(d, 'cmd_head_raw_deg'), col(d, 'cmd_heading_deg'),
        [('rotation speed [deg/s]', col(d, 'heading_rate_dps'), '#16a085'),
         ('roll [deg]', col(d, 'roll_deg'), '#c0392b')], args.deadzone))

    # --- VERTICAL AXIS (altitude) ---
    figs.append(plot_axis(
        outdir, 'vertical_axis', 'VERTICAL AXIS (ey -> altitude)', t, ey,
        [('P', col(d, 'p_term_alt'), '#27ae60'),
         ('I', col(d, 'i_term_alt'), '#f39c12'),
         ('D', col(d, 'd_term_alt'), '#8e44ad')],
        col(d, 'cmd_alt_raw_m'), col(d, 'cmd_alt_m'),
        [('current altitude [m]', col(d, 'current_alt_m'), '#2c3e50'),
         ('command altitude [m]', col(d, 'target_alt_m'), '#2980b9')], args.deadzone))

    # --- TARGET GEOMETRY + SPEED ---
    fig, ax = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    fig.suptitle('TARGET GEOMETRY and SPEED', fontsize=13, fontweight='bold')
    for name, key, c in (('horizontal coverage %', 'coverage_w_pct', '#2980b9'),
                         ('vertical coverage %', 'coverage_h_pct', '#8e44ad')):
        v = col(d, key)
        if v is not None:
            ax[0].plot(t, v, lw=0.9, label=name, color=c)
    ax[0].set_ylabel('coating [%]'); ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)
    sq = col(d, 'sqrt_area_px')
    if sq is None:
        ar = col(d, 'target_area_px')
        sq = np.sqrt(np.clip(ar, 0, None)) if ar is not None else None
    if sq is not None:
        ax[1].plot(t, sq, lw=0.9, color='#d35400', label='sqrt(box_area) [px]')
    ax[1].set_ylabel('sqrt(box_area) [px]'); ax[1].legend(fontsize=8); ax[1].grid(alpha=0.3)
    for name, key, c in (('airspeed', 'airspeed_ms', '#c0392b'),
                         ('groundspeed', 'groundspeed_ms', '#27ae60')):
        v = col(d, key)
        if v is not None:
            ax[2].plot(t, v, lw=0.9, label=name, color=c)
    thr = col(d, 'throttle_pct')
    if thr is not None:
        a2 = ax[2].twinx(); a2.plot(t, thr, lw=0.7, color='#7f8c8d', alpha=0.7, label='throttle %')
        a2.set_ylabel('throttle [%]')
    ax[2].set_ylabel('speed [m/s]'); ax[2].set_xlabel('elapsed duration [s]')
    ax[2].legend(fontsize=8, loc='upper left'); ax[2].grid(alpha=0.3)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    p = os.path.join(outdir, 'target_and_speed.png'); fig.savefig(p, dpi=110); plt.close(fig)
    figs.append(p)

    # --- STATISTICS ---
    sx = axis_stats(t, ex, col(d, 'cmd_head_raw_deg'), col(d, 'cmd_heading_deg'),
                    col(d, 'sat_head'), args.deadzone)
    sy = axis_stats(t, ey, col(d, 'cmd_alt_raw_m'), col(d, 'cmd_alt_m'),
                    col(d, 'sat_alt'), args.deadzone)

    # --- GROUND TRUTH CROSS CHECK ---
    gt_lines = []
    verdict_gt = None
    if args.ground_truth and os.path.exists(args.ground_truth):
        g = read_csv(args.ground_truth)
        tyr = col(g, 'target_yaw_rate_dps')
        man = col(g, 'target_maneuvering')
        rng = col(g, 'range_m')
        be = col(g, 'bearing_error_deg')
        if tyr is not None:
            neutral_pct = float(100.0 * np.mean(np.abs(tyr[~np.isnan(tyr)]) < 2.0))
            gt_lines.append(f"- Target rotation speed median: **{np.nanmedian(np.abs(tyr)):.2f} deg/s**; "
                            f"**{neutral_pct:.1f}%** of the time the target flies STRAIGHT (|yaw rate| < 2 deg/s)")
            if rng is not None:
                gt_lines.append(f"- range: {np.nanmin(rng):.0f}–{np.nanmax(rng):.0f} m "
                                f"(median {np.nanmedian(rng):.0f} m)")
            if be is not None:
                gt_lines.append(f"- Heading error against ground truth: mean absolute error {np.nanmean(np.abs(be)):.2f} deg, "
                                f"std {np.nanstd(be):.2f} deg")
            osc = (sx.get('mean_abs_error', 0) > 2.0) or (sx.get('cmd_std', 0) > 5.0)
            if neutral_pct > 70.0 and osc:
                verdict_gt = ("**CONTROLLER-RELATED OSCILLATION.** The target flies straight %.1f%% of the time. Horizontal "
                              "mean absolute error is %.2f deg and command std is %.2f deg. Target motion "
                              "does not explain the observed oscillation." %
                              (neutral_pct, sx.get('mean_abs_error', float('nan')), sx.get('cmd_std', float('nan'))))
            elif neutral_pct > 70.0:
                verdict_gt = ("The target is flying mostly straight and our error is small — in this arm "
                              "The controller looks healthy.")
            else:
                verdict_gt = ("The target is maneuvering most of the time; How much of the error? "
                              "how much from the controller is indistinguishable from the difficulty of tracking.")
        else:
            gt_lines.append("- WARNING: ground truth CSV does not have target_yaw_rate_dps.")

    # --- ASSESSMENT ---
    def release_value(s, sample_value, n=2, measurement_unit=' s'):
        """If the gate is not passed, write an explicit expression instead of a bare number (raw value fails)."""
        if s.get('oscillation_detected'):
            return '—' if sample_value is None else f"**{sample_value:.{n}f}{measurement_unit}**"
        gk = s.get('short_reason', 'no oscillation')
        if sample_value is None:
            return gk
        return f"{gk} — raw {sample_value:.{n}f}{measurement_unit}, NOT oscillation indicator"

    def axis_block(entry_name, s, measurement_unit):
        L = [f"### {entry_name}", "",
             f"| criterion | value | reference |", "|---|---|---|",
             f"| mean \\|error\\| | **{fmt(s.get('mean_abs_error'))} deg** | good configuration 0.44 / bad configuration 5.74 |",
             f"| error std | {fmt(s.get('error_std'))} deg | — |",
             f"| error RMS | {fmt(s.get('error_rms'))} deg | — |",
             f"| max \\|error\\| | {fmt(s.get('max_abs_error'))} deg | — |",
             f"| duration in deadzone | %{fmt(s.get('within_deadzone_pct'), 1)} | high = calm |",
             f"| oscillation amplitude (trimmed std) | {fmt(s.get('amplitude'))} deg | threshold {fmt(s.get('amplitude_threshold'))} deg (= deadzone); below = measurement noise |",
             f"| **oscillation gate** | {'**PASSED**' if s.get('oscillation_detected') else '**FAILED**'} | {s.get('reason', '—')} |",
             f"| dominant period (out-of-band direction change) | {release_value(s, s.get('zero_crossing_period'), 2, ' s')} | actual flight 4.5–6.0 s, limit-cycle ~2.0 s |",
             f"| dominant period (FFT) | {release_value(s, s.get('fft_period'), 2, ' s')} | fraction of peak power %{fmt(s.get('peak_power_pct'), 1)}, threshold %{DOMINANT_THRESHOLD_PCT:.0f} |",
             f"| sign change (hysteresis, band ±{fmt(s.get('band'))} deg) | {release_value(s, s.get('sign_changes_per_min'), 1, ' /min')} | only direction changes LEAVING the band are counted ({s.get('transition_count', 0)} units) |",
             f"| raw (non-gated) zero crossing | {fmt(s.get('raw_zero_crossing_period'))} s / {fmt(s.get('raw_sign_changes_per_min'), 1)} /min | also counts noise — for the record, NO DIAGNOSTIC VALUE |",
             f"| command std | {fmt(s.get('cmd_std'))} {measurement_unit} | actual flight 12.3–12.8 (heading) |",
             f"| command saturation | %{fmt(s.get('saturation_pct'), 1)} | high = frequent command limiting |",
             f"| sample / duration | {s.get('n', 0)} / {fmt(s.get('duration_s'), 1)} s | — |", ""]
        return L

    md = [f"# PID assessment report", "",
          f"- guidance CSV: `{args.csv}`",
          f"- Ground truth: `{args.ground_truth}`" if args.ground_truth else "- Ground truth: **N/A** (Must be collected with tools/ground_truth_logger.py)",
          f"- TRACKING rate: %{100.0 * np.mean(tracking):.1f} ({int(np.sum(tracking))}/{len(tracking)} frame)",
          ""]
    md += axis_block('HORIZONTAL AXIS (ex -> heading)', sx, 'deg')
    md += axis_block('VERTICAL AXIS (ey -> altitude)', sy, 'm')
    if gt_lines:
        md += ["### Ground truth comparison", ""] + gt_lines + [""]
    md += ["### ASSESSMENT", ""]
    md += [axis_verdict('HORIZONTAL AXIS (ex -> heading)', sx, 'deg'),
           axis_verdict('VERTICAL AXIS (ey -> altitude)', sy, 'm'), ""]
    md += [f"oscillation gate: amplitude (trimmed std) >= deadzone ({args.deadzone:.2f} deg) AND "
           f"FFT peak power >= %{DOMINANT_THRESHOLD_PCT:.0f} AND at least {MIN_TRANSITIONS} out-of-band directional change. "
           "If one of the three conditions is not met, the period and sign-change values describe "
           "measurement noise, not evidence of oscillation.", ""]
    if verdict_gt:
        md += [verdict_gt, ""]
    else:
        md += ["Ground truth was not given; 'Are we swaying while the target is going straight?' question "
               "CANNOT BE ANSWERED COMPLETELY (the axis judgment above is based solely on our own measurement). "
               "Get simultaneous recording with `tools/ground_truth_logger.py`.", ""]
    md += ["### Plots", ""] + [f"- `{os.path.basename(f)}`" for f in figs]

    rp = os.path.join(outdir, 'ASSESSMENT.md')
    with open(rp, 'w') as fh:
        fh.write("\n".join(md) + "\n")

    print("\n".join(md))
    print(f"\n-> {rp}")
    for f in figs:
        print(f"-> {f}")


if __name__ == '__main__':
    main()
