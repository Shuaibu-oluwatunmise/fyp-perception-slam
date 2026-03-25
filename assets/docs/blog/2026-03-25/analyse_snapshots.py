#!/usr/bin/env python3
"""
Speed-Dx Benchmark — Analysis
==============================
Loads all snapshot_NNN/ directories produced by record_snapshot.py,
pairs each mapped cone's dx error with the vehicle speed at the time
that cone was first localised, and plots the relationship.

Hypothesis: dx_error = speed × pipeline_latency  (linear, slope = latency in seconds)

USAGE
-----
  python3 testing/SpeedDx/analyse_snapshots.py
"""

import csv
import math
import re
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# ── Design tokens (match accuracy bench style) ────────────────────────────────
BG     = '#0d1117'
SFC    = '#161b22'
BORDER = '#30363d'
TXT    = '#e6edf3'
SUB    = '#8b949e'
BLUE   = '#58a6ff'
YEL    = '#d29922'
RED    = '#f85149'
GREEN  = '#3fb950'
ORANGE = '#ff7b72'
SNAP_COLORS = ['#58a6ff', '#3fb950', '#ff7b72', '#d29922', '#bc8cff']

BASE    = Path(__file__).parent
RESULTS = BASE / 'results'

GT_MATCH_THRESHOLD    = 3.0  # metres — max distance to pair GT cone to map cone
SPEED_LOG_MATCH_RADIUS = 2.0  # metres — position match radius for speed log lookup


# ── Parsing (identical to accuracy bench) ─────────────────────────────────────

def parse_car_pose(odom_file):
    txt = open(odom_file).read()
    p = re.findall(r'position:\s*\n\s+x:\s*([-\d.eE+]+)\s*\n\s+y:\s*([-\d.eE+]+)', txt)
    return float(p[0][0]), float(p[0][1])


def parse_car_speed(odom_file):
    """Extract instantaneous speed from odom snapshot (for reference only)."""
    txt = open(odom_file).read()
    m = re.search(r'linear:\s*\n\s+x:\s*([-\d.eE+]+)', txt)
    return float(m.group(1)) if m else None


def parse_gt(gt_file, car_x, car_y):
    out = []
    for b in re.split(r'(?=- header:)', open(gt_file).read()):
        a = re.search(r'action:\s*(\d+)', b)
        if a and int(a.group(1)) != 0:
            continue
        p = re.findall(r'position:\s*\n\s+x:\s*([-\d.eE+]+)\s*\n\s+y:\s*([-\d.eE+]+)', b)
        if not p:
            continue
        x, y = float(p[0][0]), float(p[0][1])
        if abs(x) < 0.001:
            continue
        out.append((x + car_x, y + car_y))
    return out


def parse_map(cone_file):
    """Returns list of (x, y, class_name, speed, source)."""
    out = []
    for b in re.split(r'(?=- header:)', open(cone_file).read()):
        p = re.findall(r'position:\s*\n\s+x:\s*([-\d.eE+]+)\s*\n\s+y:\s*([-\d.eE+]+)', b)
        if not p:
            continue
        cn = re.search(r'class_name:\s*(\w+)', b)
        c = cn.group(1) if cn else 'unknown'
        if c == 'unknown':
            continue
        sm = re.search(r'speed:\s*([-\d.eE+]+)', b)
        spd = float(sm.group(1)) if sm else 0.0
        src = re.search(r'source:\s*(\S+)', b)
        source = src.group(1) if src else ''
        out.append((float(p[0][0]), float(p[0][1]), c, spd, source))
    return out



def parse_speed_log(log_file):
    rows = []
    with open(log_file, newline='') as f:
        for row in csv.DictReader(f):
            try:
                rows.append({
                    'timestamp': float(row['timestamp']),
                    'speed_ms':  float(row['speed_ms']),
                    'x':         float(row['x']),
                    'y':         float(row['y']),
                    'source':    row['source'].strip(),
                })
            except (ValueError, KeyError):
                continue
    return rows


def match_speed_from_log(map_x, map_y, speed_log):
    """
    For a given global map cone position, find the speed log entry whose
    position is closest within SPEED_LOG_MATCH_RADIUS. Returns the speed
    at that first detection event (can be 0 — valid data point).
    """
    best_d, best_speed = float('inf'), None
    for row in speed_log:
        if row['source'] not in ('fused', 'lidar'):
            continue
        d = math.sqrt((map_x - row['x']) ** 2 + (map_y - row['y']) ** 2)
        if d < best_d and d < SPEED_LOG_MATCH_RADIUS:
            best_d, best_speed = d, row['speed_ms']
    return best_speed


def build_rows(gt_cones, map_cones, car_y):
    # map_cones is list of (x, y, class_name, speed, source)
    gt_L  = sorted([c for c in gt_cones if c[1] > car_y],  key=lambda t: t[0])
    map_L = sorted([m for m in map_cones if m[2] in ('blue', 'large_orange') and m[1] > -3.0],
                   key=lambda t: t[0])
    gt_R  = sorted([c for c in gt_cones if c[1] <= car_y], key=lambda t: t[0])
    map_R = sorted([m for m in map_cones if m[2] in ('yellow', 'large_orange') and m[1] <= -3.0],
                   key=lambda t: t[0])
    return gt_L, map_L, gt_R, map_R


def compute_errors(gt_row, map_row):
    """Returns dx, dy arrays and matched (map_x, map_y, speed, source) list."""
    dx_list, dy_list, matched_gt, matched_map = [], [], [], []
    used = set()
    for gx, gy in gt_row:
        best_d, best_i = float('inf'), -1
        for i, (mx, my, *_) in enumerate(map_row):
            if i in used:
                continue
            d = math.sqrt((gx - mx) ** 2 + (gy - my) ** 2)
            if d < best_d:
                best_d, best_i = d, i
        if best_i >= 0 and best_d <= GT_MATCH_THRESHOLD:
            used.add(best_i)
            mx, my, _, spd, src = map_row[best_i]
            dx_list.append(gx - mx)
            dy_list.append(gy - my)
            matched_gt.append((gx, gy))
            matched_map.append((mx, my, spd, src))
    return np.array(dx_list), np.array(dy_list), matched_gt, matched_map


# ── Plotting helpers ──────────────────────────────────────────────────────────

def _ax(ax):
    ax.set_facecolor(SFC)
    for s in ax.spines.values():
        s.set_color(BORDER)
    ax.tick_params(colors=SUB)
    ax.grid(True, color=BORDER, alpha=0.5, linewidth=0.5)


# ── Plots ─────────────────────────────────────────────────────────────────────

def plot_scatter(all_points, out_path):
    """Figure 1: Speed vs dx scatter with linear fit."""
    speeds = np.array([p['speed'] for p in all_points])
    dxs    = np.array([p['dx']    for p in all_points])
    snaps  = np.array([p['snap']  for p in all_points])

    fig, ax = plt.subplots(figsize=(10, 6))
    fig.patch.set_facecolor(BG)
    _ax(ax)

    unique_snaps = sorted(set(snaps))
    for i, s in enumerate(unique_snaps):
        mask = snaps == s
        col = SNAP_COLORS[i % len(SNAP_COLORS)]
        ax.scatter(speeds[mask], dxs[mask], c=col, s=50, alpha=0.75,
                   edgecolors='white', linewidths=0.4, label=f'Snapshot {s:03d}', zorder=3)

    # Linear fit
    if len(speeds) >= 2:
        coeffs  = np.polyfit(speeds, dxs, 1)
        slope, intercept = coeffs
        r2 = float(np.corrcoef(speeds, dxs)[0, 1] ** 2)
        fit_x = np.linspace(speeds.min(), speeds.max(), 100)
        fit_y = np.polyval(coeffs, fit_x)
        ax.plot(fit_x, fit_y, color=RED, lw=2, ls='--', zorder=4,
                label=f'Linear fit  (slope={slope*1000:.0f} ms, R²={r2:.3f})')
        ax.annotate(
            f'Implied pipeline latency: {slope*1000:.0f} ms\n'
            f'Static offset: {intercept:.3f} m\nR² = {r2:.3f}',
            xy=(0.03, 0.97), xycoords='axes fraction',
            va='top', ha='left', fontsize=10, color=TXT,
            bbox=dict(boxstyle='round,pad=0.4', facecolor=SFC, edgecolor=BORDER)
        )
        print(f'\nLinear fit: dx = {slope:.4f}·v + {intercept:.4f}')
        print(f'Implied latency: {slope*1000:.1f} ms   R²={r2:.4f}')

    ax.axhline(0, color=BORDER, lw=1)
    ax.set_xlabel('Speed at detection (m/s)', color=SUB, fontsize=11)
    ax.set_ylabel('dx error  GT − Map  (m)', color=SUB, fontsize=11)
    ax.set_title('Speed vs Longitudinal Map Error — FSAI', color=TXT,
                 fontweight='bold', fontsize=13)
    ax.legend(facecolor=SFC, edgecolor=BORDER, labelcolor=TXT, fontsize=9)
    fig.tight_layout(pad=1.5)
    fig.savefig(out_path, dpi=150, bbox_inches='tight', facecolor=BG)
    plt.close(fig)
    print(f'Saved: {out_path}')


def plot_boxplot(all_points, out_path):
    """Figure 2: Box plot of dx by 5 m/s speed bins."""
    bin_edges  = [0, 5, 10, 15, 20, 25, 35]
    bin_labels = ['0–5', '5–10', '10–15', '15–20', '20–25', '25+']
    bins = [[] for _ in bin_labels]

    for p in all_points:
        for i in range(len(bin_edges) - 1):
            if bin_edges[i] <= p['speed'] < bin_edges[i + 1]:
                bins[i].append(p['dx'])
                break

    non_empty = [(lbl, data) for lbl, data in zip(bin_labels, bins) if data]
    if not non_empty:
        print('No data for box plot.')
        return

    labels, data = zip(*non_empty)

    fig, ax = plt.subplots(figsize=(10, 6))
    fig.patch.set_facecolor(BG)
    _ax(ax)

    bp = ax.boxplot(data, labels=labels, patch_artist=True,
                    medianprops=dict(color=RED, lw=2),
                    whiskerprops=dict(color=SUB),
                    capprops=dict(color=SUB),
                    flierprops=dict(marker='o', color=SUB, markersize=4, alpha=0.5))
    for i, patch in enumerate(bp['boxes']):
        patch.set_facecolor(SNAP_COLORS[i % len(SNAP_COLORS)])
        patch.set_alpha(0.7)
        patch.set_edgecolor(BORDER)

    ax.axhline(0, color=BORDER, lw=1)
    ax.set_xlabel('Speed bin (m/s)', color=SUB, fontsize=11)
    ax.set_ylabel('dx error  GT − Map  (m)', color=SUB, fontsize=11)
    ax.set_title('dx Error Distribution by Speed Band — FSAI', color=TXT,
                 fontweight='bold', fontsize=13)

    print('\nPer-bin summary:')
    print(f'  {"Speed bin":<12} {"N":>4} {"Mean dx":>10} {"Std dx":>10}')
    for lbl, d in zip(labels, data):
        arr = np.array(d)
        print(f'  {lbl:<12} {len(arr):>4} {np.mean(arr):>10.3f} m  {np.std(arr):>8.3f} m')

    fig.tight_layout(pad=1.5)
    fig.savefig(out_path, dpi=150, bbox_inches='tight', facecolor=BG)
    plt.close(fig)
    print(f'Saved: {out_path}')


def plot_summary(snapshot_stats, out_path):
    """Figure 3: Mean dx and mean speed per snapshot run."""
    snaps  = [s['snap']       for s in snapshot_stats]
    mean_dx = [s['mean_dx']   for s in snapshot_stats]
    mean_v  = [s['mean_speed'] for s in snapshot_stats]
    n_pts   = [s['n']         for s in snapshot_stats]

    x = np.arange(len(snaps))
    fig, ax1 = plt.subplots(figsize=(8, 5))
    fig.patch.set_facecolor(BG)
    _ax(ax1)

    bars = ax1.bar(x, mean_dx, color=[SNAP_COLORS[i % len(SNAP_COLORS)] for i in range(len(snaps))],
                   edgecolor=BORDER, width=0.5, label='Mean dx (m)')
    for bar, n in zip(bars, n_pts):
        ax1.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.01,
                 f'n={n}', ha='center', va='bottom', color=SUB, fontsize=8)

    ax1.set_xticks(x)
    ax1.set_xticklabels([f'Snapshot {s:03d}' for s in snaps], color=TXT)
    ax1.set_ylabel('Mean dx error (m)', color=SUB, fontsize=11)
    ax1.set_title('Per-Run Summary — Mean dx vs Mean Speed', color=TXT,
                  fontweight='bold', fontsize=13)

    ax2 = ax1.twinx()
    ax2.set_facecolor(SFC)
    ax2.plot(x, mean_v, 'o--', color=YEL, lw=2, markersize=8, label='Mean speed (m/s)')
    ax2.set_ylabel('Mean speed at detection (m/s)', color=YEL, fontsize=11)
    ax2.tick_params(axis='y', colors=YEL)

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2,
               facecolor=SFC, edgecolor=BORDER, labelcolor=TXT, fontsize=9)

    fig.tight_layout(pad=1.5)
    fig.savefig(out_path, dpi=150, bbox_inches='tight', facecolor=BG)
    plt.close(fig)
    print(f'Saved: {out_path}')


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    snap_dirs = sorted(RESULTS.glob('snapshot_???'), key=lambda p: int(p.name.split('_')[1]))
    if not snap_dirs:
        print(f'No snapshot directories found in {RESULTS}')
        print('Run record_snapshot.py after each scenario run first.')
        sys.exit(1)

    print(f'Found {len(snap_dirs)} snapshot(s): {[d.name for d in snap_dirs]}')

    all_points     = []  # list of dicts: {speed, dx, dy, snap}
    snapshot_stats = []

    for snap_dir in snap_dirs:
        snap_n = int(snap_dir.name.split('_')[1])
        print(f'\n── Snapshot {snap_n:03d} ({snap_dir.name}) ──')

        f_gt    = snap_dir / 'gt_snapshot.txt'
        f_odom  = snap_dir / 'odom_snapshot.txt'
        f_cones = snap_dir / 'cones_snapshot.txt'

        f_log = snap_dir / 'speed_log.csv'
        for f in [f_gt, f_odom, f_cones, f_log]:
            if not f.exists():
                print(f'  [!] Missing {f.name}, skipping snapshot.')
                break
        else:
            car_x, car_y = parse_car_pose(f_odom)
            gt_cones  = parse_gt(f_gt, car_x, car_y)
            map_cones = parse_map(f_cones)
            speed_log = parse_speed_log(f_log)

            print(f'  GT: {len(gt_cones)} cones   Map: {len(map_cones)}   Speed log: {len(speed_log)} entries')

            gt_L, map_L, gt_R, map_R = build_rows(gt_cones, map_cones, car_y)
            dx_L, dy_L, _, mmap_L = compute_errors(gt_L, map_L)
            dx_R, dy_R, _, mmap_R = compute_errors(gt_R, map_R)

            points_this_snap = []
            for (mx, my, spd, src), dx, dy in zip(mmap_L + mmap_R,
                                                    np.concatenate([dx_L, dx_R]),
                                                    np.concatenate([dy_L, dy_R])):
                # Look up speed from log by position — ground truth of speed at detection time
                log_speed = match_speed_from_log(mx, my, speed_log)
                speed_to_use = log_speed if log_speed is not None else spd
                points_this_snap.append({'speed': speed_to_use, 'dx': dx, 'dy': dy, 'snap': snap_n})

            all_points.extend(points_this_snap)

            if points_this_snap:
                speeds_s = [p['speed'] for p in points_this_snap]
                dxs_s    = [p['dx']    for p in points_this_snap]
                snapshot_stats.append({
                    'snap':       snap_n,
                    'n':          len(points_this_snap),
                    'mean_dx':    float(np.mean(np.abs(dxs_s))),
                    'mean_speed': float(np.mean(speeds_s)),
                })
                print(f'  Matched {len(points_this_snap)} cone pairs with speed data. '
                      f'Mean speed={np.mean(speeds_s):.1f} m/s  Mean |dx|={np.mean(np.abs(dxs_s)):.3f} m')

    if not all_points:
        print('\nNo data points collected. Check that speed_log.csv has fused cone entries.')
        sys.exit(1)

    print(f'\nTotal data points across all snapshots: {len(all_points)}')

    plot_scatter(all_points,  RESULTS / 'speed_dx_scatter.png')
    plot_boxplot(all_points,  RESULTS / 'speed_dx_boxplot.png')
    if snapshot_stats:
        plot_summary(snapshot_stats, RESULTS / 'speed_dx_summary.png')

    print('\nDone.')


if __name__ == '__main__':
    main()
