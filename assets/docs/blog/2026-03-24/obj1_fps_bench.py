#!/usr/bin/env python3
"""
Objective 1 — FPS Benchmark  (5-run, real frames)
===================================================
Phase 1: Collect N_COLLECT real frames from /camera/image_raw via ROS2.
Phase 2: Run inference 5 times over those frames (independent passes).

Outputs saved to testing/FPS/results/:
  fps_line_per_frame.png   — per-frame latency across all 5 runs
  fps_bar_runs.png         — FPS per run + overall average
  fps_summary_card.png     — clean result summary card

Usage:
  # Pipeline must be running:
  ros2 launch fsai_perception perception.launch.py source:=carmaker

  cd ~/fsai_ws
  source install/setup.bash
  python3 testing/FPS/obj1_fps_bench.py
"""

import os
import sys
import time
import datetime
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

# ── Config ─────────────────────────────────────────────────────────────────────
MODEL_PATH  = '/home/mdxfsai/fsai_ws/src/perception/models/yolo26s/best.pt'
DEVICE      = 'cuda:0'
IMG_SIZE    = 640
CONF        = 0.5
IOU         = 0.45
N_COLLECT   = 100
N_WARMUP    = 10
N_RUNS      = 5
TARGET_FPS  = 30.0
TARGET_MS   = 1000.0 / TARGET_FPS   # 33.3 ms

RESULTS_DIR = Path(__file__).parent / 'results'

# ── Design tokens ──────────────────────────────────────────────────────────────
BG          = '#0d1117'
SURFACE     = '#161b22'
BORDER      = '#30363d'
TEXT        = '#e6edf3'
SUBTEXT     = '#8b949e'
ACCENT      = '#58a6ff'
GREEN       = '#3fb950'
RED         = '#f85149'
AMBER       = '#d29922'
RUN_COLORS  = ['#58a6ff', '#3fb950', '#d29922', '#bc8cff', '#ff7b72']


# ── Phase 1: Frame collection ──────────────────────────────────────────────────

class FrameCollector(Node):
    def __init__(self):
        super().__init__('fps_bench_collector')
        self.bridge  = CvBridge()
        self.frames  = []
        self._sub = self.create_subscription(
            Image, '/camera/image_raw', self._cb, 10)
        self.get_logger().info(
            f'Collecting {N_COLLECT} real frames from /camera/image_raw ...')

    def _cb(self, msg):
        if len(self.frames) >= N_COLLECT:
            return
        self.frames.append(
            self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'))
        n = len(self.frames)
        if n % 10 == 0:
            self.get_logger().info(f'  {n}/{N_COLLECT} frames collected...')

    def done(self):
        return len(self.frames) >= N_COLLECT


def collect_frames():
    rclpy.init()
    node = FrameCollector()
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        print('\nInterrupted.')
        sys.exit(1)
    frames = node.frames[:N_COLLECT]
    node.destroy_node()
    rclpy.shutdown()
    print(f'Collected {len(frames)} frames.\n')
    return frames


# ── Phase 2: Benchmark ─────────────────────────────────────────────────────────

def run_benchmark(frames, model):
    """Single timed pass over all frames. Returns list of per-frame ms."""
    latencies = []
    for frame in frames:
        t0 = time.perf_counter()
        model.predict(frame, conf=CONF, iou=IOU, imgsz=IMG_SIZE,
                      device=DEVICE, verbose=False)
        latencies.append((time.perf_counter() - t0) * 1000)
    return latencies


# ── Plotting ───────────────────────────────────────────────────────────────────

def _apply_style(fig):
    fig.patch.set_facecolor(BG)


def _ax_style(ax, title=None):
    ax.set_facecolor(SURFACE)
    ax.tick_params(colors=SUBTEXT, labelsize=9)
    ax.xaxis.label.set_color(SUBTEXT)
    ax.yaxis.label.set_color(SUBTEXT)
    for spine in ax.spines.values():
        spine.set_color(BORDER)
    if title:
        ax.set_title(title, color=TEXT, fontsize=11, fontweight='bold', pad=10)


def plot_line_per_frame(all_latencies, run_fps):
    fig, ax = plt.subplots(figsize=(12, 5))
    _apply_style(fig)
    _ax_style(ax, 'Per-Frame Inference Latency — 5 Runs')

    for i, lat in enumerate(all_latencies):
        ax.plot(lat, color=RUN_COLORS[i], alpha=0.85, linewidth=1.2,
                label=f'Run {i+1}  ({run_fps[i]:.1f} FPS)')

    ax.axhline(TARGET_MS, color=RED, linestyle='--', linewidth=1.5,
               label=f'30 FPS threshold ({TARGET_MS:.1f} ms)')

    ax.set_xlabel('Frame index', fontsize=10)
    ax.set_ylabel('Inference latency (ms)', fontsize=10)
    ax.legend(facecolor=SURFACE, edgecolor=BORDER, labelcolor=TEXT,
              fontsize=9, loc='upper right')
    ax.grid(True, color=BORDER, linewidth=0.5, alpha=0.6)

    fig.tight_layout(pad=1.5)
    out = RESULTS_DIR / 'fps_line_per_frame.png'
    fig.savefig(out, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'  Saved: {out}')


def plot_bar_runs(run_fps):
    fig, ax = plt.subplots(figsize=(8, 5))
    _apply_style(fig)
    _ax_style(ax, 'FPS per Run — YOLO26s on Jetson (cuda:0)')

    xs = np.arange(1, N_RUNS + 1)
    bars = ax.bar(xs, run_fps, color=RUN_COLORS[:N_RUNS],
                  width=0.55, zorder=3, edgecolor=BORDER, linewidth=0.8)

    avg = np.mean(run_fps)
    ax.axhline(avg, color=ACCENT, linestyle='--', linewidth=1.8,
               label=f'Average  {avg:.1f} FPS', zorder=4)
    ax.axhline(TARGET_FPS, color=RED, linestyle=':', linewidth=1.5,
               label=f'Target  {TARGET_FPS:.0f} FPS', zorder=4)

    for bar, fps in zip(bars, run_fps):
        ax.text(bar.get_x() + bar.get_width() / 2,
                bar.get_height() + 0.4,
                f'{fps:.1f}', ha='center', va='bottom',
                color=TEXT, fontsize=10, fontweight='bold')

    ax.set_xticks(xs)
    ax.set_xticklabels([f'Run {i}' for i in xs])
    ax.set_ylabel('FPS', fontsize=10)
    ax.set_ylim(0, max(run_fps) * 1.2)
    ax.legend(facecolor=SURFACE, edgecolor=BORDER, labelcolor=TEXT, fontsize=9)
    ax.grid(True, axis='y', color=BORDER, linewidth=0.5, alpha=0.6, zorder=0)

    fig.tight_layout(pad=1.5)
    out = RESULTS_DIR / 'fps_bar_runs.png'
    fig.savefig(out, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'  Saved: {out}')


def plot_summary_card(run_fps, run_ms):
    avg_fps = np.mean(run_fps)
    avg_ms  = np.mean(run_ms)
    passed  = avg_fps >= TARGET_FPS
    status_color = GREEN if passed else RED
    status_text  = 'PASS' if passed else 'FAIL'
    timestamp    = datetime.datetime.now().strftime('%Y-%m-%d  %H:%M')

    fig = plt.figure(figsize=(9, 6))
    _apply_style(fig)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_facecolor(BG)
    ax.axis('off')

    def box(x, y, w, h, color=SURFACE, radius=0.02):
        fancy = mpatches.FancyBboxPatch(
            (x, y), w, h,
            boxstyle=f'round,pad={radius}',
            facecolor=color, edgecolor=BORDER, linewidth=1.5,
            transform=ax.transAxes, clip_on=False)
        ax.add_patch(fancy)

    # Header bar
    box(0.03, 0.84, 0.94, 0.13, color='#161b22')
    ax.text(0.50, 0.915, 'OBJECTIVE 1 — FPS BENCHMARK',
            ha='center', va='center', transform=ax.transAxes,
            color=TEXT, fontsize=14, fontweight='bold',
            fontfamily='monospace')
    ax.text(0.50, 0.862, 'YOLO26s  ·  NVIDIA Jetson  ·  cuda:0  ·  PIONEER FSAI',
            ha='center', va='center', transform=ax.transAxes,
            color=SUBTEXT, fontsize=9)

    # Big FPS + PASS badge
    box(0.03, 0.53, 0.43, 0.28)
    ax.text(0.245, 0.695, f'{avg_fps:.1f}',
            ha='center', va='center', transform=ax.transAxes,
            color=ACCENT, fontsize=52, fontweight='bold')
    ax.text(0.245, 0.595, 'avg FPS',
            ha='center', va='center', transform=ax.transAxes,
            color=SUBTEXT, fontsize=11)

    box(0.48, 0.53, 0.49, 0.28)
    ax.text(0.725, 0.715, status_text,
            ha='center', va='center', transform=ax.transAxes,
            color=status_color, fontsize=44, fontweight='bold')
    ax.text(0.725, 0.595,
            f'Target  ≥ {TARGET_FPS:.0f} FPS   ·   avg latency {avg_ms:.1f} ms/frame',
            ha='center', va='center', transform=ax.transAxes,
            color=SUBTEXT, fontsize=9.5)

    # Per-run table
    box(0.03, 0.08, 0.94, 0.42)
    col_x  = [0.10, 0.30, 0.50, 0.68, 0.85]
    headers = ['Run', 'FPS', 'Latency (ms)', 'vs Target', 'Status']
    for hx, hd in zip(col_x, headers):
        ax.text(hx, 0.465, hd,
                ha='center', va='center', transform=ax.transAxes,
                color=SUBTEXT, fontsize=9, fontweight='bold')

    ax.plot([0.04, 0.96], [0.455, 0.455],
            color=BORDER, linewidth=0.8, transform=ax.transAxes)

    for i, (fps, ms) in enumerate(zip(run_fps, run_ms)):
        row_y = 0.395 - i * 0.065
        delta = fps - TARGET_FPS
        d_col = GREEN if delta >= 0 else RED
        d_str = f'+{delta:.1f}' if delta >= 0 else f'{delta:.1f}'
        s_col = GREEN if fps >= TARGET_FPS else RED
        s_str = 'PASS' if fps >= TARGET_FPS else 'FAIL'
        vals  = [f'Run {i+1}', f'{fps:.1f}', f'{ms:.1f}', d_str, s_str]
        cols  = [TEXT,        TEXT,          TEXT,        d_col,  s_col]
        for hx, v, c in zip(col_x, vals, cols):
            ax.text(hx, row_y, v,
                    ha='center', va='center', transform=ax.transAxes,
                    color=c, fontsize=10,
                    fontweight='bold' if hx in (col_x[-1],) else 'normal')

    # Footer
    ax.text(0.97, 0.025, timestamp,
            ha='right', va='center', transform=ax.transAxes,
            color=SUBTEXT, fontsize=8)
    ax.text(0.03, 0.025, f'{N_COLLECT} frames  ·  {N_RUNS} runs  ·  {N_WARMUP} warmup',
            ha='left', va='center', transform=ax.transAxes,
            color=SUBTEXT, fontsize=8)

    out = RESULTS_DIR / 'fps_summary_card.png'
    fig.savefig(out, dpi=180, bbox_inches='tight', facecolor=BG)
    plt.close(fig)
    print(f'  Saved: {out}')


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    print('=== Phase 1: Collecting real frames from CarMaker ===')
    frames = collect_frames()

    print('=== Phase 2: Loading model ===')
    print(f'  {MODEL_PATH}')
    model = YOLO(MODEL_PATH)

    print(f'\nWarming up ({N_WARMUP} runs)...')
    for _ in range(N_WARMUP):
        model.predict(frames[0], conf=CONF, iou=IOU, imgsz=IMG_SIZE,
                      device=DEVICE, verbose=False)

    print(f'\n=== Phase 3: Benchmarking ({N_RUNS} runs × {N_COLLECT} frames) ===')
    all_latencies = []
    run_fps = []
    run_ms  = []

    for r in range(N_RUNS):
        lats = run_benchmark(frames, model)
        fps  = 1000.0 / np.mean(lats)
        ms   = np.mean(lats)
        all_latencies.append(lats)
        run_fps.append(fps)
        run_ms.append(ms)
        print(f'  Run {r+1}: {fps:.1f} FPS  ({ms:.1f} ms/frame)')

    avg_fps = np.mean(run_fps)
    avg_ms  = np.mean(run_ms)
    status  = 'PASS' if avg_fps >= TARGET_FPS else 'FAIL'

    print(f'\n  Overall avg: {avg_fps:.1f} FPS  —  [{status}]')

    print('\n=== Saving outputs ===')
    plot_line_per_frame(all_latencies, run_fps)
    plot_bar_runs(run_fps)
    plot_summary_card(run_fps, run_ms)
    print(f'\nAll outputs saved to: {RESULTS_DIR}')


if __name__ == '__main__':
    main()
