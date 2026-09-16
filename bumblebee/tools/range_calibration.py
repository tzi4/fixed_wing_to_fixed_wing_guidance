#!/usr/bin/env python3
r"""
Offline range calibration from apparent target size.

The camera-only guidance experiment cannot read target position during
operation. Offline calibration supplies a range estimate for its
range-dependent vertical aim offset. For a fixed-size target with image
size s pixels, s is approximately k/R, giving R approximately k/s.

Fit k from paired guidance and ground-truth logs produced by
tools/ground_truth_logger.py, then save the model to JSON. Guidance can
use the calibration with camera data alone.

Fit three candidate models and evaluate them on a held-out time segment.
Randomly splitting consecutive, nearly identical frames would leak
information between training and test data. Report explicitly when a
model fails to generalize.

Usage:
    python3 tools/range_calibration.py --pair guidance.csv:ground_truth.csv
    python3 tools/range_calibration.py --pair a_guidance.csv:a_ground_truth.csv         --pair b_guidance.csv:b_ground_truth.csv
    python3 tools/range_calibration.py --pair g.csv:r.csv --max-difference 0.10         --json reports/range_calibration.json
"""
import argparse
import csv
import json
import os
import sys
import time

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# range boxes: near / middle / far (m). The error is reported separately on each of these because the 1/s model is inherently noisier at longer range.
BBOXES = (('near (<100 m)', 0.0, 100.0),
           ('medium (100-300 m)', 100.0, 300.0),
           ('far (>300 m)', 300.0, float('inf')))

MIN_SAMPLES = 10      # fitting is unreliable below this sample count
MIN_TEST_SAMPLES = 5        # If the retained test set is smaller than this, assessment is not given.


# ------------------------------------------------------------------------------ Reading CSV (same approach as pid_plot.py) ------------------------------------------------------------------------------
def read_csv_file(path):
    if not os.path.exists(path):
        sys.exit(f"ERROR: file not found: {path}")
    with open(path, newline='') as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        sys.exit(f"ERROR: {path} is empty (can only be a header row).")
    cols = {}
    for k in rows[0]:
        vals = []
        for r in rows:
            v = (r.get(k) or '').strip()
            try:
                vals.append(float(v))
            except ValueError:
                vals.append(np.nan)
        cols[k] = np.array(vals, dtype=float)
    cols['_raw'] = rows
    return cols


def column_name(d, name, default=None):
    """Spin your shot; default if not present or all NaN."""
    v = d.get(name)
    if v is None or len(v) == 0 or np.all(np.isnan(v)):
        return default
    return v


def time_interval(t):
    g = t[np.isfinite(t)]
    if len(g) == 0:
        return None, None
    return float(np.min(g)), float(np.max(g))


# ----------------------------------------------------------------------------- Join a guidance + ground truth pair by CLOSEST timestamp -----------------------------------------------------------------------------
def process_pair(guidance_path, ground_truth_path, max_difference):
    g = read_csv_file(guidance_path)
    r = read_csv_file(ground_truth_path)

    tg = column_name(g, 'timestamp')
    if tg is None:
        sys.exit(f"ERROR: Missing 'timestamp' (unix) shot in {guidance_path} — "
                 "is this a guidance CSV? Cannot merge with elapsed_s.")
    tr = column_name(r, 'timestamp')
    if tr is None:
        sys.exit(f"ERROR: Missing 'timestamp' (unix) shot in {ground_truth_path} — "
                 "is this a ground_truth_logger.py CSV?")

    # --- visible size: sqrt_area_px otherwise derive from target_area_px ---
    s = column_name(g, 'sqrt_area_px')
    size_source = 'sqrt_area_px'
    if s is None:
        box_area = column_name(g, 'target_area_px')
        if box_area is None:
            sys.exit(f"ERROR: Neither 'sqrt_area_px' nor {guidance_path} "
                     "There is 'target_area_px'; apparent size cannot be calculated.")
        s = np.sqrt(np.clip(box_area, 0.0, None))
        size_source = 'sqrt(target_area_px)'
    bw = column_name(g, 'bbox_w')
    bh = column_name(g, 'bbox_h')
    if bw is None or bh is None:
        sys.exit(f"ERROR: There is no shot bbox_w/bbox_h in {guidance_path}.")

    state_value = np.array([(row.get('system_state') or '').strip() for row in g['_raw']])
    tracking_state = (state_value == 'TRACKING')

    valid_sample = (tracking_state & np.isfinite(tg) & np.isfinite(s) & np.isfinite(bw) &
               np.isfinite(bh) & (s > 0) & (bw > 0) & (bh > 0))
    n_raw = len(tg)
    n_valid = int(np.sum(valid_sample))

    # --- ground truth range: range_m or from range_xy_m + alt_diff_m ---
    R = column_name(r, 'range_m')
    range_source = 'range_m'
    if R is None:
        rxy = column_name(r, 'range_xy_m')
        dz = column_name(r, 'alt_diff_m')
        if rxy is None or dz is None:
            sys.exit(f"ERROR: {ground_truth_path} does not contain 'range_m' and "
                     "It also cannot be calculated with range_xy_m/alt_diff_m.")
        R = np.hypot(rxy, dz)
        range_source = 'hypot(range_xy_m, alt_diff_m)'

    range_valid = np.isfinite(tr) & np.isfinite(R) & (R > 0)
    tr_g, R_g = tr[range_valid], R[range_valid]
    layout_spec = np.argsort(tr_g)
    tr_g, R_g = tr_g[layout_spec], R_g[layout_spec]

    summary = {
        'guidance_csv': os.path.abspath(guidance_path),
        'ground_truth_csv': os.path.abspath(ground_truth_path),
        'guidance_rows': n_raw,
        'valid_tracking_rows': n_valid,
        'ground_truth_rows': int(len(tr_g)),
        'size_source': size_source,
        'range_source': range_source,
        'guidance_time': time_interval(tg),
        'ground_truth_time': time_interval(tr_g),
        'matched_samples': 0,
        'time_difference_rejections': 0,
    }

    empty_value = np.array([])
    if n_valid == 0 or len(tr_g) == 0:
        return empty_value, empty_value, empty_value, empty_value, summary

    tg_s, s_s, bw_s, bh_s = tg[valid_sample], s[valid_sample], bw[valid_sample], bh[valid_sample]

    # --- CLOSEST timestamp match (searchsorted; compare earlier and later samples) ---
    idx = np.searchsorted(tr_g, tg_s)
    left_value = np.clip(idx - 1, 0, len(tr_g) - 1)
    right_value = np.clip(idx, 0, len(tr_g) - 1)
    d_left = np.abs(tg_s - tr_g[left_value])
    d_right = np.abs(tg_s - tr_g[right_value])
    nearest_candidate = np.where(d_left <= d_right, left_value, right_value)
    difference = np.minimum(d_left, d_right)

    keep_mask = difference <= max_difference
    summary['matched_samples'] = int(np.sum(keep_mask))
    summary['time_difference_rejections'] = int(np.sum(~keep_mask))
    if summary['matched_samples']:
        summary['median_time_difference_s'] = float(np.median(difference[keep_mask]))

    return tg_s[keep_mask], s_s[keep_mask], bh_s[keep_mask], R_g[nearest_candidate[keep_mask]], summary


# ----------------------------------------------------------------------------- Models — all least squares -----------------------------------------------------------------------------
def inverse_fit(x, R):
    """R = k / x -> single-parameter least squares passing through the origin."""
    u = 1.0 / x
    denominator = float(np.dot(u, u))
    if not np.isfinite(denominator) or denominator <= 0:
        return None
    return {'k': float(np.dot(u, R) / denominator)}


def inverse_fit_offset(x, R):
    """R = a/x + b (two-parameter least squares)."""
    A = np.column_stack([1.0 / x, np.ones_like(x)])
    if not np.all(np.isfinite(A)):
        return None
    try:
        c, *_ = np.linalg.lstsq(A, R, rcond=None)
    except np.linalg.LinAlgError:
        return None
    return {'a': float(c[0]), 'b': float(c[1])}


MODELS = [
    {'entry_name': 'A', 'formula': 'R = k / sqrt_area_px', 'input_column': 'sqrt_area_px',
     'fit': inverse_fit, 'predict': lambda p, x: p['k'] / x},
    {'entry_name': 'B', 'formula': 'R = k / bbox_h', 'input_column': 'bbox_h',
     'fit': inverse_fit, 'predict': lambda p, x: p['k'] / x},
    {'entry_name': 'C', 'formula': 'R = a / sqrt_area_px + b', 'input_column': 'sqrt_area_px',
     'fit': inverse_fit_offset, 'predict': lambda p, x: p['a'] / x + p['b']},
]


def metrics(R_true, R_est):
    """R^2, RMSE (m), median absolute percent error + range box break."""
    m = np.isfinite(R_true) & np.isfinite(R_est)
    R_true, R_est = R_true[m], R_est[m]
    out = {'n': int(len(R_true))}
    if len(R_true) == 0:
        return out
    art = R_est - R_true
    ss_res = float(np.sum(art ** 2))
    ss_tot = float(np.sum((R_true - np.mean(R_true)) ** 2))
    out['r2'] = (1.0 - ss_res / ss_tot) if ss_tot > 1e-12 else None
    out['rmse_m'] = float(np.sqrt(np.mean(art ** 2)))
    out['median_absolute_percent'] = float(np.median(np.abs(art) / R_true * 100.0))
    out['range_bins'] = {}
    for entry_name, lo, hi in BBOXES:
        k = (R_true >= lo) & (R_true < hi)
        if not np.any(k):
            out['range_bins'][entry_name] = {'n': 0}
            continue
        out['range_bins'][entry_name] = {
            'n': int(np.sum(k)),
            'rmse_m': float(np.sqrt(np.mean(art[k] ** 2))),
            'median_absolute_percent': float(np.median(np.abs(art[k]) / R_true[k] * 100.0)),
        }
    return out


def format_spec(v, n=2, extra_text=''):
    if v is None or (isinstance(v, float) and not np.isfinite(v)):
        return '—'
    return f"{v:.{n}f}{extra_text}"


def parameter_text(p):
    if not p:
        return '—'
    return ', '.join(f"{k} = {v:.4f}" for k, v in p.items())


def metric_row(label, m):
    if not m or m.get('n', 0) == 0:
        return f"    {label:<13} no sample"
    return (f"    {label:<13} n={m['n']:<6d} R2={format_spec(m.get('r2'), 4):<9} "
            f"RMSE={format_spec(m.get('rmse_m'), 1)} m   "
            f"median |error|=%{format_spec(m.get('median_absolute_percent'), 1)}")


def bbox_rows(m):
    L = []
    for entry_name, _, _ in BBOXES:
        k = (m.get('range_bins') or {}).get(entry_name, {})
        if k.get('n', 0) == 0:
            L.append(f"      {entry_name:<18} no sample")
        else:
            L.append(f"      {entry_name:<18} n={k['n']:<6d} RMSE={format_spec(k['rmse_m'], 1)} m   "
                     f"median |error|=%{format_spec(k['median_absolute_percent'], 1)}")
    return L


# --------------------------------------------------------------------------
# Plot
# --------------------------------------------------------------------------
def plot_graph(png_path, s, bh, R, results, selected):
    fig, ax = plt.subplots(3, 1, figsize=(12, 12))
    fig.suptitle('range CALIBRATION — apparent size vs ground truth range',
                 fontsize=13, fontweight='bold')
    plot_color = {'A': '#c0392b', 'B': '#27ae60', 'C': '#8e44ad'}

    # --- 1: range vs sqrt(area) + A and C curves ---
    ax[0].scatter(s, R, s=6, alpha=0.35, color='#2980b9', label='measurement (ground truth)')
    xs = np.linspace(max(float(np.min(s)), 1e-6), float(np.max(s)), 400)
    for so in results:
        if so['input_column'] != 'sqrt_area_px' or so['all_parameters'] is None:
            continue
        ax[0].plot(xs, so['predict'](so['all_parameters'], xs), lw=1.6, color=plot_color[so['entry_name']],
                   label=f"{so['entry_name']}: {so['formula']}  ({parameter_text(so['all_parameters'])})")
    ax[0].set_xlabel('sqrt(box_area) [px]'); ax[0].set_ylabel('range [m]')
    ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)

    # --- 2: range vs bbox_h + B curve ---
    ax[1].scatter(bh, R, s=6, alpha=0.35, color='#16a085', label='measurement (ground truth)')
    xh = np.linspace(max(float(np.min(bh)), 1e-6), float(np.max(bh)), 400)
    for so in results:
        if so['input_column'] != 'bbox_h' or so['all_parameters'] is None:
            continue
        ax[1].plot(xh, so['predict'](so['all_parameters'], xh), lw=1.6, color=plot_color[so['entry_name']],
                   label=f"{so['entry_name']}: {so['formula']}  ({parameter_text(so['all_parameters'])})")
    ax[1].set_xlabel('bbox_h [px]'); ax[1].set_ylabel('range [m]')
    ax[1].legend(fontsize=8); ax[1].grid(alpha=0.3)

    # --- 3: residual (prediction - actual) vs actual range ---
    for so in results:
        if so['all_parameters'] is None:
            continue
        x = s if so['input_column'] == 'sqrt_area_px' else bh
        art = so['predict'](so['all_parameters'], x) - R
        emphasis = (so['entry_name'] == selected)
        ax[2].scatter(R, art, s=9 if emphasis else 4, alpha=0.55 if emphasis else 0.20,
                      color=plot_color[so['entry_name']], label=f"{so['entry_name']}{' (SELECTED)' if emphasis else ''}")
    ax[2].axhline(0, color='k', lw=0.8)
    for _, lo, hi in BBOXES:
        if np.isfinite(hi) and hi < float(np.max(R)):
            ax[2].axvline(hi, color='#7f8c8d', lw=0.7, ls='--')
    ax[2].set_xlabel('ground truth range [m]')
    ax[2].set_ylabel('residual (estimate - actual) [m]')
    ax[2].legend(fontsize=8); ax[2].grid(alpha=0.3)

    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(png_path, dpi=110)
    plt.close(fig)


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description='OFFLINE calibration for range estimation from apparent size')
    ap.add_argument('--pair', action='append', default=[], metavar='GUIDANCE.csv:GROUND_TRUTH.csv',
                    help='Ground truth CSV pair with guidance CSV (repeatable)')
    ap.add_argument('--max-difference', type=float, default=0.15,
                    help='maximum timestamp difference [s] for match (default 0.15)')
    ap.add_argument('--test-ratio', type=float, default=0.30,
                    help='Test margin retained from end of TIME (default 0.30)')
    ap.add_argument('--json', default=os.path.join('reports', 'range_calibration.json'),
                    help='output path JSON (PNG is written to the same name)')
    args = ap.parse_args()

    if not args.pair:
        sys.exit("ERROR: at least one --pair GUIDANCE.csv:GROUND_TRUTH.csv required.\n"
                 "      Example: --pair flight_logs/flight_log_guided_X.csv:"
                 "flight_logs/ground_truth_X.csv")
    if not (0.05 <= args.test_ratio <= 0.9):
        sys.exit("ERROR: --test-rate must be between 0.05 and 0.90.")
    if args.max_difference <= 0:
        sys.exit("ERROR: --max-difference must be positive.")

    print("=" * 78)
    print("range CALIBRATION")
    print("=" * 78)

    # --- read pairs and combine by timestamp ---
    S, BH, RR, GROUP = [], [], [], []
    summaries = []
    for i, p in enumerate(args.pair):
        if ':' not in p:
            sys.exit(f"ERROR: --pair format should be 'guidance.csv:ground_truth.csv', received: {p}")
        guidance_path, ground_truth_path = p.rsplit(':', 1)
        _tg, s, bh, R, summary = process_pair(guidance_path, ground_truth_path, args.max_difference)
        summaries.append(summary)
        print(f"\n[pair {i + 1}] {os.path.basename(guidance_path)} + {os.path.basename(ground_truth_path)}")
        print(f"    guidance line: {summary['guidance_rows']}")
        print(f"    TRACKING + valid_sample bbox : {summary['valid_tracking_rows']}")
        print(f"    ground truth row       : {summary['ground_truth_rows']}")
        print(f"    matched_samples (<= {args.max_difference:.3f} s)  : {summary['matched_samples']}")
        print(f"    thrown (time difference): {summary['time_difference_rejections']}")
        if summary.get('median_time_difference_s') is not None:
            print(f"    median time difference: {summary['median_time_difference_s'] * 1000:.1f} ms")
        print(f"    size/range source: {summary['size_source']}/{summary['range_source']}")
        if summary['matched_samples'] == 0:
            g0, g1 = summary['guidance_time']
            r0, r1 = summary['ground_truth_time']
            print("    WARNING: no rows from this pair matched.")
            if g0 is not None and r0 is not None:
                shared_data = min(g1, r1) - max(g0, r0)
                print(f"           guidance window: {g0:.1f} .. {g1:.1f}")
                print(f"           real window: {r0:.1f}..{r1:.1f}")
                if shared_data <= 0:
                    print(f"           two records DO NOT overlap (there are {-shared_data:.1f} s between them) — "
                          "These two files are not from the same flight.")
                else:
                    print(f"           {shared_data:.1f} s have overlap; --maximum-difference can be enlarged.")
            continue
        S.append(s); BH.append(bh); RR.append(R)
        GROUP.append(np.full(len(s), i, dtype=int))

    if not S:
        print("\nRESULT: not a single matching row; Calibration COULD NOT BE DONE.")
        print("       Guidance and ground truth recordings are from the SAME flight and simultaneously "
              "It should be.")
        sys.exit(2)

    s_all = np.concatenate(S); bh_all = np.concatenate(BH)
    R_all = np.concatenate(RR); group_name = np.concatenate(GROUP)
    n = len(s_all)
    print(f"\n Total matching sample: {n}")
    print(f"range: {np.min(R_all):.1f} .. {np.max(R_all):.1f} m "
          f"(median {np.median(R_all):.1f} m)")
    print(f"sqrt(box_area) range   : {np.min(s_all):.2f} .. {np.max(s_all):.2f} px")
    print(f"bbox_h range       : {np.min(bh_all):.2f} .. {np.max(bh_all):.2f} px")

    if n < MIN_SAMPLES:
        print(f"\nRESULT: No fitting with sample {n} (minimum {MIN_SAMPLES}). "
              "Longer concurrent recording required.")
        sys.exit(3)

    # --- TIME separation: first 70% of each pair is fit, last 30% is test --- Random separation LEAKS: consecutive frames are nearly identical data.
    fit_mask = np.zeros(n, dtype=bool)
    for i in np.unique(group_name):
        k = np.where(group_name == i)[0]
        if len(k) == 1:
            fit_mask[k] = True
            continue
        split_index = int(round(len(k) * (1.0 - args.test_ratio)))
        split_index = max(1, min(len(k) - 1, split_index))
        fit_mask[k[:split_index]] = True
    test_mask = ~fit_mask
    n_fit, n_test = int(np.sum(fit_mask)), int(np.sum(test_mask))
    print(f"\nSeparation by time: fit {n_fit} example | sample held test {n_test} "
          f"(last %{args.test_ratio * 100:.0f} of each pair)")
    test_valid = n_test >= MIN_TEST_SAMPLES
    if not test_valid:
        print(f"    WARNING: retained test set is smaller than sample {MIN_TEST_SAMPLES}; "
              "A generalization judgment CANNOT be made.")

    inputs = {'sqrt_area_px': s_all, 'bbox_h': bh_all}

    # --- fit the models ---
    results = []
    print("\n" + "-" * 78)
    print("MODELS")
    print("-" * 78)
    for m in MODELS:
        x = inputs[m['input_column']]
        all_p = m['fit'](x, R_all)
        fit_p = m['fit'](x[fit_mask], R_all[fit_mask]) if n_fit else None
        so = {
            'entry_name': m['entry_name'], 'formula': m['formula'], 'input_column': m['input_column'],
            'predict': m['predict'], 'all_parameters': all_p, 'fit_param': fit_p,
            'all_samples': metrics(R_all, m['predict'](all_p, x)) if all_p else {},
            'in_sample': metrics(R_all[fit_mask], m['predict'](fit_p, x[fit_mask])) if fit_p else {},
            'test': (metrics(R_all[test_mask], m['predict'](fit_p, x[test_mask]))
                     if (fit_p and n_test > 0) else {}),
        }
        results.append(so)

        print(f"\n  Model {so['entry_name']}: {so['formula']}")
        if all_p is None:
            print("    fit failed (singular matrix or invalid input).")
            continue
        print(f"    All data parameters: {parameter_text(all_p)}")
        print(f"    fit-set parameters: {parameter_text(fit_p)}")
        print(metric_row('ALL DATA', so['all_samples']))
        print("      range boxes (all data):")
        for row in bbox_rows(so['all_samples']):
            print(row)
        print(metric_row('INTERNAL SAMPLING', so['in_sample']))
        print(metric_row('HELD-OUT TEST', so['test']))
        if so['test'].get('n', 0):
            print("      range boxes (retained test):")
            for row in bbox_rows(so['test']):
                print(row)

    # ---choose the best model according to the TEST HELD RMSE ---
    candidate_item = [so for so in results if so['all_parameters'] is not None]
    if not candidate_item:
        print("\nRESULT: no model could be fitted.")
        sys.exit(4)

    measurable = []
    if test_valid:
        measurable = [so for so in candidate_item if so['test'].get('rmse_m') is not None]
        selection_criterion = 'held-out test RMSE'
    if not measurable:
        measurable = [so for so in candidate_item if so['all_samples'].get('rmse_m') is not None]
        selection_criterion = 'all data RMSE (retained test unreliable)'
        test_valid = False
    if not measurable:
        print("\nRESULT: models could not be compared.")
        sys.exit(4)
    best_candidate = min(measurable,
                 key=lambda so: (so['test'] if test_valid else so['all_samples'])['rmse_m'])

    print("\n" + "=" * 78)
    print(f"SELECTED MODEL: {best_candidate['entry_name']} — {best_candidate['formula']}")
    print(f"Selection criteria: {selection_criterion}")
    print(f"Parameters (refitted to ALL data): {parameter_text(best_candidate['all_parameters'])}")
    print(f"Valid range: {np.min(R_all):.1f} .. {np.max(R_all):.1f} m "
          "(extrapolation outside this range is unreliable)")
    print("=" * 78)

    # --- generalization clause: clearly as follows ---
    internal_rmse = best_candidate['in_sample'].get('rmse_m')
    test_rmse = best_candidate['test'].get('rmse_m')
    test_r2 = best_candidate['test'].get('r2')
    if not test_valid:
        generalization = ("Insufficient test set retained — whether the model generalizes "
                     "NOT MEASURED. This calibration can be trusted as much as fitting error, "
                     "not more.")
    elif internal_rmse is None or test_rmse is None:
        generalization = "Generalization could not be measured (metric could not be calculated)."
    elif test_r2 is not None and test_r2 < 0:
        generalization = (f"MODEL DOES NOT GENERALIZE: retained test R2 = {test_r2:.3f} < 0, i.e. prediction "
                     "worse than the constant mean estimate. This calibration is done at runtime. "
                     "MUST_NOT_BE_USED.")
    elif internal_rmse > 1e-9 and test_rmse > 2.0 * internal_rmse:
        generalization = (f"WARNING: retained test RMSE ({test_rmse:.1f} m) internal sample RMSE "
                     f"({internal_rmse:.1f} m) More than 2 times larger. The pattern glides across the record; "
                     "A single constant may not be enough for the entire flight.")
    else:
        generalization = (f"The model generalizes: retained test RMSE {test_rmse:.1f} m, internal sample "
                     f"{internal_rmse:.1f}m; Median absolute error in retained test "
                     f"%{format_spec(best_candidate['test'].get('median_absolute_percent'), 1)}.")
    print("\nGENELLEME: " + generalization)

    # --- JSON + PNG ---
    json_path = os.path.abspath(args.json)
    os.makedirs(os.path.dirname(json_path) or '.', exist_ok=True)
    png_path = os.path.splitext(json_path)[0] + '.png'

    def clean_samples(so):
        return {'entry_name': so['entry_name'], 'formula': so['formula'], 'input_column': so['input_column'],
                'all_data_parameters': so['all_parameters'],
                'training_parameters': so['fit_param'],
                'all_data': so['all_samples'], 'training_set': so['in_sample'],
                'held_out_test': so['test']}

    result_output = {
        'created_at': time.strftime('%Y-%m-%dT%H:%M:%S'),
        'description': ('Range estimation from apparent size. guidance code shows the location of the target '
                     'CANNOT READ; these coefficients were calibrated offline.'),
        'model': best_candidate['entry_name'],
        'formula': best_candidate['formula'],
        'input_column': best_candidate['input_column'],
        'parameters': best_candidate['all_parameters'],
        'selection_criterion': selection_criterion,
        'generalization_verdict': generalization,
        'sample_count': int(n),
        'fit_samples': n_fit,
        'test_samples': n_test,
        'test_ratio': args.test_ratio,
        'max_time_difference_s': args.max_difference,
        'fit_statistics': {
            'all_data': best_candidate['all_samples'],
            'training_set': best_candidate['in_sample'],
            'held_out_test': best_candidate['test'],
        },
        'valid_range_interval': {
            'min_m': float(np.min(R_all)),
            'max_m': float(np.max(R_all)),
            'median_m': float(np.median(R_all)),
            'note': 'Fit is only valid in this range; EXTRAPOLATION outside is unreliable.',
        },
        'valid_size_interval': {
            'sqrt_area_px_min': float(np.min(s_all)),
            'sqrt_area_px_max': float(np.max(s_all)),
            'bbox_h_min': float(np.min(bh_all)),
            'bbox_h_max': float(np.max(bh_all)),
        },
        'source_csv_files': summaries,
        'all_models': [clean_samples(so) for so in results],
    }
    with open(json_path, 'w') as fh:
        json.dump(result_output, fh, indent=2, ensure_ascii=False)

    plot_graph(png_path, s_all, bh_all, R_all, results, best_candidate['entry_name'])

    print(f"\n-> {json_path}")
    print(f"-> {png_path}")


if __name__ == '__main__':
    main()
