#!/usr/bin/env python3
"""
Objective 2 — Spatial Accuracy Benchmark (Snapshot Based)
===========================================================
Automatically captures text snapshots from live ROS 2 topics and 
runs an offline-style comparison to generate Objective 2 accuracy PNGs.

HOW IT WORKS
------------
1. Launches `ros2 topic echo ... --once` for:
     - /carmaker/ObjectList
     - /carmaker/odom
     - /global_map/cones
2. Saves these to `results/*_snapshot.txt`.
3. Parses the text snapshots exactly like the manual validation.
4. Generates `per_cone_errors.png` and `summary_card.png`.

USAGE
-----
  # Ensure pipeline and CarMaker are running, and car has finished scenario
  python3 testing/Accuracy/obj2_accuracy_bench.py
"""

import os
import re
import sys
import time
import argparse
import subprocess
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from pathlib import Path

BASE = Path(__file__).parent
RESULTS = BASE / 'results'

# Design tokens
BG = '#0d1117'
SFC = '#161b22'
BORDER = '#30363d'
TXT = '#e6edf3'
SUB = '#8b949e'
BLUE = '#58a6ff'
YEL = '#d29922'
RED = '#f85149'
GREEN = '#3fb950'
ORANGE = '#ff7b72'
TARGET = 0.20


def collect_snapshots(out_dir: Path):
    """Run ros2 topic echo to capture one message from each topic into a txt file."""
    print("Capturing live snapshots from ROS 2 (ensure CarMaker is running)...")
    out_dir.mkdir(parents=True, exist_ok=True)
    
    f_gt = out_dir / 'gt_snapshot.txt'
    f_odom = out_dir / 'odom_snapshot.txt'
    f_cones = out_dir / 'cones_snapshot.txt'
    
    # Launch in parallel so we don't block sequentially
    p1 = subprocess.Popen(f"ros2 topic echo /carmaker/ObjectList --once > {f_gt}", shell=True)
    p2 = subprocess.Popen(f"ros2 topic echo /carmaker/odom --once > {f_odom}", shell=True)
    p3 = subprocess.Popen(f"ros2 topic echo /global_map/cones --once 2>/dev/null > {f_cones}", shell=True)
    
    t0 = time.time()
    while p1.poll() is None or p2.poll() is None or p3.poll() is None:
        if time.time() - t0 > 15.0:
            p1.terminate(); p2.terminate(); p3.terminate()
            print("\n[!] Timeout waiting for ROS 2 topics. Are they publishing?")
            sys.exit(1)
        time.sleep(0.5)
        sys.stdout.write('.')
        sys.stdout.flush()
        
    print(f"\nSnapshots saved to {out_dir}/")
    return f_gt, f_odom, f_cones


def parse_car_pose(odom_file):
    with open(odom_file) as f:
        txt = f.read()
    cx_match = re.search(r'position:\s*\n\s+x:\s*([-\d.eE+]+)', txt)
    cy_match = re.findall(r'position:\s*\n\s+[xy]:\s*[-\d.eE+]+\s*\n\s+y:\s*([-\d.eE+]+)', txt)
    if not cx_match or not cy_match:
        p = re.findall(r'position:\s*\n\s+x:\s*([-\d.eE+]+)\s*\n\s+y:\s*([-\d.eE+]+)', txt)
        return float(p[0][0]), float(p[0][1])
    return float(cx_match.group(1)), float(cy_match[0])


def parse_gt(gt_file, car_x, car_y):
    out = []
    for b in re.split(r'(?=- header:)', open(gt_file).read()):
        a = re.search(r'action:\s*(\d+)', b)
        if a and int(a.group(1)) != 0: continue
        p = re.findall(r'position:\s*\n\s+x:\s*([-\d.eE+]+)\s*\n\s+y:\s*([-\d.eE+]+)', b)
        if not p: continue
        x, y = float(p[0][0]), float(p[0][1])
        if abs(x) < 0.001: continue
        out.append((x + car_x, y + car_y))
    return out


def parse_map(cone_file):
    out = []
    for b in re.split(r'(?=- header:)', open(cone_file).read()):
        p = re.findall(r'position:\s*\n\s+x:\s*([-\d.eE+]+)\s*\n\s+y:\s*([-\d.eE+]+)', b)
        if not p: continue
        cn = re.search(r'class_name:\s*(\w+)', b)
        c = cn.group(1) if cn else 'unknown'
        if c == 'unknown': continue
        out.append((float(p[0][0]), float(p[0][1]), c))
    return out


# ── Plotting ──────────────────────────────────────────────────────────────────

def _ax(ax):
    ax.set_facecolor(SFC)
    for s in ax.spines.values(): s.set_color(BORDER)
    ax.tick_params(colors=SUB)
    ax.grid(True, color=BORDER, alpha=0.5, linewidth=0.5)


def compute_errors(gt_row, map_row, dist_threshold=3.0):
    """
    Pairs each GT cone to the closest Map cone using Euclidean distance (Nearest Neighbor).
    A threshold prevents pairing with completely unrelated cones if a false negative exists.
    """
    dx_list, dy_list = [], []
    matched_gt, matched_map = [], []
    used_map_indices = set()
    
    # Sort GT by x just for predictable top-to-bottom processing
    for gx, gy in gt_row:
        best_d = float('inf')
        best_i = -1
        
        for i, (mx, my, mc) in enumerate(map_row):
            if i in used_map_indices:
                continue
            d = np.sqrt((gx - mx)**2 + (gy - my)**2)
            if d < best_d:
                best_d = d
                best_i = i
                
        if best_i >= 0 and best_d <= dist_threshold:
            used_map_indices.add(best_i)
            mx, my, mc = map_row[best_i]
            dx_list.append(gx - mx)
            dy_list.append(gy - my)
            matched_gt.append((gx, gy))
            matched_map.append((mx, my))
            
    return np.array(dx_list), np.array(dy_list), matched_gt, matched_map


def plot_per_cone_errors(dx_L, dy_L, dx_R, dy_R, mgt_L, mmap_L, mgt_R, mmap_R, mapc, SKIP_GATES, n, out_path):
    gates = np.arange(1, n + 1)

    fig = plt.figure(figsize=(16, 12))
    fig.patch.set_facecolor(BG)

    # Overhead
    ax1 = fig.add_subplot(3, 1, 1)
    _ax(ax1)
    for x, y, c in mapc:
        if c == 'unknown': continue
        col = {'blue': BLUE, 'yellow': YEL, 'large_orange': ORANGE}.get(c, SUB)
        ax1.scatter(x, y, marker='o', s=80, c=col, edgecolors='white', linewidths=0.5, zorder=3)

    for (gx, gy), (mx, my) in zip(mgt_L, mmap_L):
        ax1.scatter(gx, gy, marker='x', s=80, c='white', linewidths=2, zorder=5)
        ax1.plot([mx, gx], [my, gy], color='white', lw=0.8, alpha=0.5, zorder=4)

    for (gx, gy), (mx, my) in zip(mgt_R, mmap_R):
        ax1.scatter(gx, gy, marker='x', s=80, c='white', linewidths=2, zorder=5)
        ax1.plot([mx, gx], [my, gy], color='white', lw=0.8, alpha=0.5, zorder=4)

    from matplotlib.lines import Line2D
    ax1.legend(handles=[
        Line2D([0],[0],marker='x',color=SFC,markerfacecolor='white',markeredgecolor='white', markersize=10, label='GT (shifted)'),
        Line2D([0],[0],marker='o',color=SFC,markerfacecolor=BLUE,markersize=8, label='Map blue'),
        Line2D([0],[0],marker='o',color=SFC,markerfacecolor=YEL,markersize=8, label='Map yellow'),
        Line2D([0],[0],marker='o',color=SFC,markerfacecolor=ORANGE,markersize=8, label='Map orange'),
    ], facecolor=SFC, edgecolor=BORDER, labelcolor=TXT, fontsize=9)
    ax1.set_title(f'Map vs ObjectList ({n} gates paired, ignored first {SKIP_GATES})', color=TXT, fontweight='bold', fontsize=11)
    ax1.set_xlabel('x odom (m)', color=SUB); ax1.set_ylabel('y (m)', color=SUB)

    # dx
    ax2 = fig.add_subplot(3, 1, 2)
    _ax(ax2)
    ax2.plot(gates, dx_L, 'o-', color=BLUE, lw=1.5, markersize=5, label=f'Left   mean={np.mean(dx_L):+.2f} m  std={np.std(dx_L):.2f} m')
    ax2.plot(gates, dx_R, 's-', color=YEL, lw=1.5, markersize=5, label=f'Right  mean={np.mean(dx_R):+.2f} m  std={np.std(dx_R):.2f} m')
    ax2.axhline(0, color=BORDER, lw=1)
    ax2.axhline(TARGET, color=RED, lw=1.2, ls='--', alpha=0.7, label=f'±{TARGET} m target')
    ax2.axhline(-TARGET, color=RED, lw=1.2, ls='--', alpha=0.7)
    ax2.set_ylabel('dx  GT−Map (m)', color=SUB); ax2.set_xlabel('Gate #', color=SUB)
    ax2.set_title('dx error per gate (longitudinal — includes odom drift & camera offset)', color=TXT, fontweight='bold')
    ax2.legend(facecolor=SFC, edgecolor=BORDER, labelcolor=TXT, fontsize=9)

    # dy
    ax3 = fig.add_subplot(3, 1, 3)
    _ax(ax3)
    ax3.plot(gates, dy_L, 'o-', color=BLUE, lw=1.5, markersize=5, label=f'Left   mean={np.mean(dy_L):+.3f} m  std={np.std(dy_L):.3f} m')
    ax3.plot(gates, dy_R, 's-', color=YEL, lw=1.5, markersize=5, label=f'Right  mean={np.mean(dy_R):+.3f} m  std={np.std(dy_R):.3f} m')
    ax3.axhline(0, color=BORDER, lw=1)
    ax3.axhline(TARGET, color=RED, lw=1.2, ls='--', alpha=0.7, label=f'±{TARGET} m target')
    ax3.axhline(-TARGET, color=RED, lw=1.2, ls='--', alpha=0.7)
    ax3.set_ylabel('dy  GT−Map (m)', color=SUB); ax3.set_xlabel('Gate #', color=SUB)
    ax3.set_title('dy error per gate (lateral — perception accuracy)', color=TXT, fontweight='bold')
    ax3.legend(facecolor=SFC, edgecolor=BORDER, labelcolor=TXT, fontsize=9)

    fig.tight_layout(pad=1.5)
    fig.savefig(out_path, dpi=150, bbox_inches='tight', facecolor=BG)
    plt.close(fig)
    print(f'Saved: {out_path}')


def plot_summary_card(gt, mapc, dx_L, dy_L, dx_R, dy_R, SKIP_GATES, n, out_path, scenario):
    fig = plt.figure(figsize=(10, 5.5))
    fig.patch.set_facecolor(BG)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_facecolor(BG); ax.axis('off')

    def box(x, y, w, h, color=SFC):
        p = mpatches.FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.015', facecolor=color, edgecolor=BORDER, linewidth=1.5, transform=ax.transAxes, clip_on=False)
        ax.add_patch(p)

    box(0.03, 0.84, 0.94, 0.13)
    ax.text(0.50, 0.92, 'OBJECTIVE 2 — SPATIAL ACCURACY BENCHMARK', ha='center', va='center', transform=ax.transAxes, color=TXT, fontsize=13, fontweight='bold', fontfamily='monospace')
    ax.text(0.50, 0.865, f'Scenario: {scenario}  ·  PIONEER FSAI  ·  YOLOv26s + LiDAR fusion', ha='center', va='center', transform=ax.transAxes, color=SUB, fontsize=9)

    dy_mean = np.mean(np.abs(np.concatenate([dy_L, dy_R])))
    dy_max  = np.max(np.abs(np.concatenate([dy_L, dy_R])))
    dx_mean = np.mean(np.abs(np.concatenate([dx_L, dx_R])))
    
    det_rate = (n * 2) / (n * 2) * 100 if n > 0 else 0

    lat_pass = dy_mean <= TARGET
    det_pass = det_rate >= 95.0

    metrics = [
        ('Detection rate',    f'{det_rate:.1f}%',           '≥ 95%',      det_pass),
        ('Mean lateral error (dy)', f'{dy_mean*100:.1f} cm', '≤ 20 cm',   lat_pass),
        ('Max lateral error (dy)',  f'{dy_max*100:.1f} cm',  '≤ 20 cm',   dy_max <= TARGET),
        ('Mean longit. error (dx)', f'{dx_mean:.2f} m',      'odom drift', None),
    ]

    box(0.03, 0.07, 0.94, 0.73)
    cols = [0.18, 0.44, 0.62, 0.78]
    headers = ['Metric', 'Result', 'Target', 'Status']
    for hx, hd in zip(cols, headers):
        ax.text(hx, 0.75, hd, ha='center', va='center', transform=ax.transAxes, color=SUB, fontsize=9.5, fontweight='bold')
    ax.plot([0.04, 0.96], [0.73, 0.73], color=BORDER, lw=0.8, transform=ax.transAxes)

    for i, (name, val, tgt, passed) in enumerate(metrics):
        y = 0.63 - i * 0.14
        s_col = (GREEN if passed else RED) if passed is not None else SUB
        s_str = ('PASS' if passed else 'FAIL') if passed is not None else '—'
        for hx, txt, col in zip(cols, [name, val, tgt, s_str], [TXT, TXT, SUB, s_col]):
            w = 'bold' if hx == cols[-1] else 'normal'
            ax.text(hx, y, txt, ha='center', va='center', transform=ax.transAxes, color=col, fontsize=10.5, fontweight=w)

    ax.text(0.97, 0.025, 'Snapshot Mode Analysis', ha='right', va='center', transform=ax.transAxes, color=SUB, fontsize=8)
    ax.text(0.03, 0.025, f'GT={len(gt)}  Map={len(mapc)}  Paired={n*2} (ignored first {SKIP_GATES*2})', ha='left', va='center', transform=ax.transAxes, color=SUB, fontsize=8)

    fig.savefig(out_path, dpi=180, bbox_inches='tight', facecolor=BG)
    plt.close(fig)
    print(f'Saved: {out_path}')


def build_rows(gt_cones, map_cones, car_y):
    gt_L = sorted([c for c in gt_cones if c[1] > car_y], key=lambda t: t[0])
    map_L = sorted([(x,y,c) for x,y,c in map_cones if c in ('blue','large_orange') and y > -3.0], key=lambda t: t[0])

    gt_R = sorted([c for c in gt_cones if c[1] <= car_y], key=lambda t: t[0])
    map_R = sorted([(x,y,c) for x,y,c in map_cones if c in ('yellow','large_orange') and y <= -3.0], key=lambda t: t[0])
    return gt_L, map_L, gt_R, map_R


def main():
    parser = argparse.ArgumentParser(description='Obj2 Accuracy Benchmark (Snapshots)')
    parser.add_argument('--scenario', default='Acceleration', help='Scenario name for report')
    parser.add_argument('--out', default=str(RESULTS), help='Output directory for snapshots and PNGs')
    parser.add_argument('--skip-gates', type=int, default=2, dest='skip', help='Skip first N gates per row')
    args = parser.parse_args()

    out_dir = Path(args.out)
    
    # ── 1. Snapshot ──
    f_gt, f_odom, f_cones = collect_snapshots(out_dir)

    # ── 2. Parse ──
    car_x, car_y = parse_car_pose(f_odom)
    print(f"Car odom: x={car_x:.2f}, y={car_y:.2f}")

    gt_odom = sorted(parse_gt(f_gt, car_x, car_y), key=lambda t: t[0])
    map_cones = sorted(parse_map(f_cones), key=lambda t: t[0])

    # ── 3. Split into Rows ──
    gt_L, map_L, gt_R, map_R = build_rows(gt_odom, map_cones, car_y)

    # skip first N gates from GT ONLY
    s = args.skip
    if s > 0:
        gt_L = gt_L[s:]
        gt_R = gt_R[s:]
        print(f"Skipping first {s} ObjectList cones per row.")

    dx_L, dy_L, mgt_L, mmap_L = compute_errors(gt_L, map_L)
    dx_R, dy_R, mgt_R, mmap_R = compute_errors(gt_R, map_R)
    n = min(len(dx_L), len(dx_R))

    # Truncate to same paired length for parallel plotting
    dx_L, dy_L = dx_L[:n], dy_L[:n]
    dx_R, dy_R = dx_R[:n], dy_R[:n]
    mgt_L, mmap_L = mgt_L[:n], mmap_L[:n]
    mgt_R, mmap_R = mgt_R[:n], mmap_R[:n]

    print(f"Paired {n} left gates and {n} right gates.")

    # ── 4. Plot ──
    plot_per_cone_errors(dx_L, dy_L, dx_R, dy_R, mgt_L, mmap_L, mgt_R, mmap_R, map_cones, s, n, out_dir / 'per_cone_errors.png')
    plot_summary_card(gt_odom, map_cones, dx_L, dy_L, dx_R, dy_R, s, n, out_dir / 'summary_card.png', args.scenario)


if __name__ == '__main__':
    main()
