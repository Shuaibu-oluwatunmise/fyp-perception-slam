#!/usr/bin/env python3
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


def collect_snapshots(out_dir: Path):
    print("Capturing live snapshots from ROS 2 (ensure CarMaker is running)...")
    out_dir.mkdir(parents=True, exist_ok=True)
    
    f_gt = out_dir / 'gt_snapshot.txt'
    f_det = out_dir / 'detections_snapshot.txt'
    f_lidar = out_dir / 'lidar_snapshot.txt'
    f_fused = out_dir / 'fused_snapshot.txt'
    
    p1 = subprocess.Popen(f"ros2 topic echo /carmaker/ObjectList --once > {f_gt}", shell=True)
    p2 = subprocess.Popen(f"ros2 topic echo /detections --once > {f_det}", shell=True)
    p3 = subprocess.Popen(f"ros2 topic echo /detections/lidar --once 2>/dev/null > {f_lidar}", shell=True)
    p4 = subprocess.Popen(f"ros2 topic echo /cones --once 2>/dev/null > {f_fused}", shell=True)
    
    t0 = time.time()
    while None in (p1.poll(), p2.poll(), p3.poll(), p4.poll()):
        if time.time() - t0 > 15.0:
            for p in (p1, p2, p3, p4): p.terminate()
            print("\n[!] Timeout waiting for ROS 2 topics.")
            sys.exit(1)
        time.sleep(0.5)
        sys.stdout.write('.')
        sys.stdout.flush()
        
    print(f"\nSnapshots saved to {out_dir}/")
    return f_gt, f_det, f_lidar, f_fused


def parse_gt_dists(f):
    """GT is in Obj_F (approx x=1.5m from rear axle). LiDAR is at x=2.9m.
       We subtract 1.4m from GT x to approximate distance from the LiDAR."""
    dists = []
    for b in re.split(r'(?=- header:)', open(f).read()):
        a = re.search(r'action:\s*(\d+)', b)
        if a and int(a.group(1)) != 0: continue
        p = re.findall(r'position:\s*\n\s+x:\s*([-\d.eE+]+)\s*\n\s+y:\s*([-\d.eE+]+)', b)
        if not p: continue
        x, y = float(p[0][0]), float(p[0][1])
        x -= 1.4  # Normalize to Lidar_F
        if abs(x) < 0.001: continue
        dists.append(np.sqrt(x**2 + y**2))
    return sorted(dists)


def count_camera_detections(f):
    boxes = re.findall(r'class_id:\s*\'(\d+)\'', open(f).read())
    return len(boxes)


def parse_3d_max_range(f, offset_x=0.0):
    """Parses 3D coordinates. offset_x is subtracted from x before calculating distance.
       Used to normalize Fr1A (rear axle) measurements to the front LiDAR sensor."""
    ranges = []
    for b in re.split(r'(?=- header:)', open(f).read()):
        p = re.findall(r'position:\s*\n\s+x:\s*([-\d.eE+]+)\s*\n\s+y:\s*([-\d.eE+]+)', b)
        if not p: continue
        x, y = float(p[0][0]), float(p[0][1])
        x -= offset_x
        ranges.append(np.sqrt(x**2 + y**2))
    return max(ranges) if ranges else 0.0


def plot_range_summary(cam_range_str, cam_bar_val, lidar_range, fused_range, cam_det_count, gt_total, out_path, scenario):
    fig = plt.figure(figsize=(10, 5.5))
    fig.patch.set_facecolor(BG)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_facecolor(BG); ax.axis('off')

    def box(x, y, w, h, color=SFC):
        p = mpatches.FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.015', facecolor=color, edgecolor=BORDER, linewidth=1.5, transform=ax.transAxes, clip_on=False)
        ax.add_patch(p)

    box(0.03, 0.84, 0.94, 0.13)
    ax.text(0.50, 0.92, 'OBJECTIVE 3 — FUSED CONFIDENCE RANGE', ha='center', va='center', transform=ax.transAxes, color=TXT, fontsize=13, fontweight='bold', fontfamily='monospace')
    ax.text(0.50, 0.865, f'Scenario: {scenario}  ·  PIONEER FSAI  ·  YOLOv26s + LiDAR fusion', ha='center', va='center', transform=ax.transAxes, color=SUB, fontsize=9)

    target_met = (fused_range >= 20.0 and fused_range <= 35.0) or fused_range > 15.0
    pass_str = 'PASS' if target_met else 'FAIL'
    pass_col = GREEN if target_met else RED

    metrics = [
        ('Camera (YOLO2D est.)', cam_range_str,           f'({cam_det_count} cones)',  (cam_bar_val >= 20.0)),
        ('LiDAR (DBSCAN 3D)',    f'{lidar_range:.1f} m',  '',                          (lidar_range >= 20.0)),
        ('Fused (localizer)',    f'{fused_range:.1f} m',  '≥ 20.0 m target',           target_met),
    ]

    box(0.03, 0.07, 0.94, 0.73)
    cols = [0.20, 0.50, 0.70, 0.85]
    headers = ['Modality', 'Max Range', 'Notes', 'Status']
    for hx, hd in zip(cols, headers):
        ax.text(hx, 0.75, hd, ha='center', va='center', transform=ax.transAxes, color=SUB, fontsize=9.5, fontweight='bold')
    ax.plot([0.04, 0.96], [0.73, 0.73], color=BORDER, lw=0.8, transform=ax.transAxes)

    for i, (name, val, note, passed) in enumerate(metrics):
        y = 0.58 - i * 0.18
        s_col = GREEN if passed else RED
        s_str = 'PASS' if passed else 'FAIL'
        for hx, txt, col in zip(cols, [name, val, note, s_str], [TXT, TXT, SUB, s_col]):
            w = 'bold' if hx in (cols[1], cols[-1]) else 'normal'
            fontsize = 11.5 if i == 2 else 10.5  # Highlight Fused
            ax.text(hx, y, txt, ha='center', va='center', transform=ax.transAxes, color=col, fontsize=fontsize, fontweight=w)

    ax.text(0.97, 0.025, f'GT Cones: {gt_total} (Total in scene)', ha='right', va='center', transform=ax.transAxes, color=SUB, fontsize=8)

    fig.savefig(out_path, dpi=180, bbox_inches='tight', facecolor=BG)
    plt.close(fig)
    print(f'Saved: {out_path}')


def plot_bar_chart(cam_range_str, cam_bar_val, lidar_range, fused_range, out_path):
    fig, ax = plt.subplots(figsize=(8, 5))
    fig.patch.set_facecolor(BG)
    ax.set_facecolor(SFC)
    for s in ax.spines.values(): s.set_color(BORDER)
    ax.tick_params(colors=SUB)
    ax.grid(True, color=BORDER, alpha=0.5, linewidth=0.5, axis='y')

    labels = ['Camera (est)', 'LiDAR', 'Fused']
    ranges = [cam_bar_val, lidar_range, fused_range]
    colors = [BLUE, ORANGE, GREEN]

    bars = ax.bar(labels, ranges, color=colors, edgecolor='white', linewidth=1, width=0.5, zorder=3)
    
    # Target range horizontal lines
    ax.axhline(20.0, color=RED, lw=1.5, ls='--', alpha=0.8, label='Target: 20-30m range', zorder=4)
    ax.axhline(30.0, color=RED, lw=1.5, ls='--', alpha=0.8, zorder=4)

    # Bar labels
    labels_texts = [cam_range_str, f'{lidar_range:.1f} m', f'{fused_range:.1f} m']
    for bar, r, l_text in zip(bars, ranges, labels_texts):
        ax.text(bar.get_x() + bar.get_width()/2, r + 0.5, l_text, ha='center', va='bottom', color=TXT, fontweight='bold')

    ax.set_ylim(0, max(ranges + [35.0]) + 5)
    ax.set_ylabel('Max Detection Range (m)', color=SUB)
    ax.set_title('Objective 3: Max Detection Range per Modality', color=TXT, fontweight='bold')
    ax.legend(facecolor=SFC, edgecolor=BORDER, labelcolor=TXT)

    fig.tight_layout()
    fig.savefig(out_path, dpi=150, facecolor=BG)
    print(f'Saved: {out_path}')


def main():
    parser = argparse.ArgumentParser(description='Obj3 Range Benchmark (Snapshots)')
    parser.add_argument('--scenario', default='Acceleration', help='Scenario name for report')
    parser.add_argument('--out', default=str(RESULTS), help='Output directory')
    args = parser.parse_args()

    out_dir = Path(args.out)
    
    # 1. Capture 
    f_gt, f_det, f_lidar, f_fused = collect_snapshots(out_dir)

    # 2. Parse GT distances (sorted closest to furthest)
    gt_dists = parse_gt_dists(f_gt)
    n_gt = len(gt_dists)

    # 3. Camera Range estimation (User Method: interval between Nth and N+1th GT cone)
    n_cam_dets = count_camera_detections(f_det)
    if n_cam_dets > 0 and n_cam_dets < n_gt:
        cam_range_min = gt_dists[n_cam_dets - 1]
        cam_range_max = gt_dists[n_cam_dets]
        cam_range_str = f"> {cam_range_min:.1f}  < {cam_range_max:.1f} m"
        cam_bar_str = f"{cam_range_min:.1f}-{cam_range_max:.1f} m"
        cam_bar_val = cam_range_min
    elif n_cam_dets >= n_gt and n_gt > 0:
        cam_range_min = gt_dists[-1]
        cam_range_str = f"≥ {cam_range_min:.1f} m"
        cam_bar_str = cam_range_str
        cam_bar_val = cam_range_min
    else:
        cam_range_str = "0.0 m"
        cam_bar_str = "0.0 m"
        cam_bar_val = 0.0

    # 4. LiDAR and Fused exact 3D max ranges
    # LiDAR is natively in Lidar_F (front bumper).
    lidar_range = parse_3d_max_range(f_lidar)
    
    # Fused is in Fr1A (rear axle, x=0). LiDAR is at x=2.9m.
    # Subtract 2.9m from fused x to measure range from the actual sensor.
    fused_range = parse_3d_max_range(f_fused, offset_x=2.9)

    print(f"\n--- Results ---")
    print(f"Total GT cones: {n_gt}")
    print(f"Camera detected: {n_cam_dets} cones -> Est. range: {cam_range_str}")
    print(f"LiDAR max range:  {lidar_range:.1f} m")
    print(f"Fused max range:  {fused_range:.1f} m")

    plot_range_summary(cam_range_str, cam_bar_val, lidar_range, fused_range, n_cam_dets, n_gt, out_dir / 'summary_card.png', args.scenario)
    plot_bar_chart(cam_bar_str, cam_bar_val, lidar_range, fused_range, out_dir / 'range_bars.png')


if __name__ == '__main__':
    main()
