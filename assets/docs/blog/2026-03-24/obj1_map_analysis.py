#!/usr/bin/env python3
"""
Objective 1 — mAP@50 Model Comparison
=======================================
Reads results.csv from all 4 trained models and produces:

  map50_comparison.png      — mAP@50 training curves, all models
  precision_comparison.png  — Precision training curves, all models
  recall_comparison.png     — Recall training curves, all models
  summary_card.png          — Final epoch leaderboard with PASS/FAIL

YOLO26s is highlighted as the deployed/chosen model throughout.

Usage:
  cd ~/fsai_ws
  python3 testing/mAP/obj1_map_analysis.py

No ROS2 or CarMaker required — reads CSV files only.
"""

from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import datetime

# ── Paths ──────────────────────────────────────────────────────────────────────
BASE       = Path(__file__).parent
MODEL_BASE = Path('/home/mdxfsai/fsai_ws/testing/Model_Resullts/train')
RESULTS    = BASE / 'results'

MODELS = {
    'YOLO26s': {
        'csv':     BASE / 'assets/yolo26s/results.csv',
        'label':   'YOLO26s (deployed)',
        'color':   '#58a6ff',
        'lw':      2.8,
        'alpha':   1.0,
        'zorder':  5,
        'marker':  'o',
        'ms':      3,
    },
    'YOLO26n': {
        'csv':     MODEL_BASE / 'cone_detector_yolo26n/results.csv',
        'label':   'YOLO26n',
        'color':   '#8b949e',
        'lw':      1.4,
        'alpha':   0.75,
        'zorder':  3,
        'marker':  None,
        'ms':      0,
    },
    'YOLO11n': {
        'csv':     MODEL_BASE / 'cone_detector_yolo11n/results.csv',
        'label':   'YOLO11n',
        'color':   '#8b949e',
        'lw':      1.4,
        'alpha':   0.6,
        'zorder':  2,
        'marker':  None,
        'ms':      0,
    },
    'YOLOv8n': {
        'csv':     MODEL_BASE / 'cone_detector_yolov8n/results.csv',
        'label':   'YOLOv8n',
        'color':   '#8b949e',
        'lw':      1.4,
        'alpha':   0.45,
        'zorder':  1,
        'marker':  None,
        'ms':      0,
    },
}

# ── Design tokens ──────────────────────────────────────────────────────────────
BG      = '#0d1117'
SURFACE = '#161b22'
BORDER  = '#30363d'
TEXT    = '#e6edf3'
SUBTEXT = '#8b949e'
GREEN   = '#3fb950'
RED     = '#f85149'
ACCENT  = '#58a6ff'
TARGET  = 0.85
TARGET_MS = 1000.0 / 30.0

# ── CSV column indices ─────────────────────────────────────────────────────────
# epoch, time, box_loss, cls_loss, dfl_loss, precision, recall, mAP50, mAP50-95, ...
COL_EPOCH     = 0
COL_PRECISION = 5
COL_RECALL    = 6
COL_MAP50     = 7


def load_csv(path):
    data = np.loadtxt(path, delimiter=',', skiprows=1)
    return {
        'epoch':     data[:, COL_EPOCH],
        'precision': data[:, COL_PRECISION],
        'recall':    data[:, COL_RECALL],
        'map50':     data[:, COL_MAP50],
    }


def _ax_style(ax, title, ylabel, ylim=None):
    ax.set_facecolor(SURFACE)
    ax.tick_params(colors=SUBTEXT, labelsize=9)
    ax.xaxis.label.set_color(SUBTEXT)
    ax.yaxis.label.set_color(SUBTEXT)
    for spine in ax.spines.values():
        spine.set_color(BORDER)
    ax.set_title(title, color=TEXT, fontsize=12, fontweight='bold', pad=10)
    ax.set_xlabel('Epoch', fontsize=10)
    ax.set_ylabel(ylabel, fontsize=10)
    ax.grid(True, color=BORDER, linewidth=0.5, alpha=0.6)
    if ylim:
        ax.set_ylim(*ylim)


def plot_metric(metric_key, title, ylabel, target_value, fname, data_map):
    fig, ax = plt.subplots(figsize=(11, 5.5))
    fig.patch.set_facecolor(BG)
    _ax_style(ax, title, ylabel, ylim=(0.4, 1.02))

    for key, cfg in MODELS.items():
        d = data_map[key]
        ax.plot(
            d['epoch'], d[metric_key],
            color=cfg['color'], linewidth=cfg['lw'],
            alpha=cfg['alpha'], zorder=cfg['zorder'],
            label=cfg['label'],
            marker=cfg['marker'] if cfg['marker'] else '',
            markersize=cfg['ms'], markevery=10,
        )

    if target_value is not None:
        ax.axhline(target_value, color=RED, linestyle='--', linewidth=1.6,
                   label=f'Target  {target_value*100:.0f}%', zorder=6)

    # Annotate final YOLO26s value
    final_val = data_map['YOLO26s'][metric_key][-1]
    final_ep  = data_map['YOLO26s']['epoch'][-1]
    ax.annotate(
        f'  {final_val*100:.1f}%',
        xy=(final_ep, final_val),
        color=ACCENT, fontsize=10, fontweight='bold', va='center',
    )

    ax.legend(facecolor=SURFACE, edgecolor=BORDER, labelcolor=TEXT,
              fontsize=9, loc='lower right')
    fig.tight_layout(pad=1.5)
    out = RESULTS / fname
    fig.savefig(out, dpi=150, bbox_inches='tight', facecolor=BG)
    plt.close(fig)
    print(f'  Saved: {out}')


def plot_summary_card(data_map):
    fig = plt.figure(figsize=(10, 6))
    fig.patch.set_facecolor(BG)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_facecolor(BG)
    ax.axis('off')

    def box(x, y, w, h, color=SURFACE):
        p = mpatches.FancyBboxPatch(
            (x, y), w, h, boxstyle='round,pad=0.015',
            facecolor=color, edgecolor=BORDER, linewidth=1.5,
            transform=ax.transAxes, clip_on=False)
        ax.add_patch(p)

    # Header
    box(0.03, 0.85, 0.94, 0.12)
    ax.text(0.50, 0.93, 'OBJECTIVE 1 — mAP@50 MODEL COMPARISON',
            ha='center', va='center', transform=ax.transAxes,
            color=TEXT, fontsize=13, fontweight='bold', fontfamily='monospace')
    ax.text(0.50, 0.875, 'YOLO26s vs YOLO26n vs YOLO11n vs YOLOv8n  ·  100 epochs  ·  PIONEER FSAI',
            ha='center', va='center', transform=ax.transAxes,
            color=SUBTEXT, fontsize=9)

    # Table header
    box(0.03, 0.08, 0.94, 0.74)
    cols_x = [0.12, 0.30, 0.46, 0.61, 0.76, 0.90]
    headers = ['Model', 'mAP@50', 'Precision', 'Recall', 'vs Target', 'Status']
    for hx, hd in zip(cols_x, headers):
        ax.text(hx, 0.775, hd, ha='center', va='center',
                transform=ax.transAxes,
                color=SUBTEXT, fontsize=9.5, fontweight='bold')

    ax.plot([0.04, 0.96], [0.755, 0.755],
            color=BORDER, linewidth=0.8, transform=ax.transAxes)

    model_order = ['YOLO26s', 'YOLO26n', 'YOLO11n', 'YOLOv8n']
    for i, key in enumerate(model_order):
        cfg  = MODELS[key]
        d    = data_map[key]
        m50  = d['map50'][-1]
        prec = d['precision'][-1]
        rec  = d['recall'][-1]
        diff = m50 - TARGET
        d_str   = f'+{diff*100:.1f}%' if diff >= 0 else f'{diff*100:.1f}%'
        d_col   = GREEN if diff >= 0 else RED
        s_str   = 'PASS' if m50 >= TARGET else 'FAIL'
        s_col   = GREEN if m50 >= TARGET else RED
        deployed = key == 'YOLO26s'

        row_y = 0.68 - i * 0.15
        row_bg = '#1c2333' if deployed else SURFACE

        # Row background for deployed model
        if deployed:
            box(0.04, row_y - 0.055, 0.92, 0.10, color=row_bg)

        name_col = ACCENT if deployed else TEXT
        vals = [cfg['label'], f'{m50*100:.1f}%', f'{prec*100:.1f}%',
                f'{rec*100:.1f}%', d_str, s_str]
        v_cols = [name_col, TEXT, TEXT, TEXT, d_col, s_col]
        for hx, v, vc in zip(cols_x, vals, v_cols):
            weight = 'bold' if deployed or hx == cols_x[-1] else 'normal'
            ax.text(hx, row_y, v, ha='center', va='center',
                    transform=ax.transAxes,
                    color=vc, fontsize=10.5, fontweight=weight)



    # Footer
    ts = datetime.datetime.now().strftime('%Y-%m-%d  %H:%M')
    ax.text(0.97, 0.025, ts, ha='right', va='center',
            transform=ax.transAxes, color=SUBTEXT, fontsize=8)
    ax.text(0.03, 0.025, f'Target mAP@50 ≥ {TARGET*100:.0f}%',
            ha='left', va='center', transform=ax.transAxes, color=SUBTEXT, fontsize=8)

    out = RESULTS / 'summary_card.png'
    fig.savefig(out, dpi=180, bbox_inches='tight', facecolor=BG)
    plt.close(fig)
    print(f'  Saved: {out}')


def main():
    RESULTS.mkdir(parents=True, exist_ok=True)

    print('Loading CSV files...')
    data_map = {}
    for key, cfg in MODELS.items():
        data_map[key] = load_csv(cfg['csv'])
        final = data_map[key]['map50'][-1]
        print(f'  {key:10s}  mAP@50={final*100:.1f}%  '
              f'P={data_map[key]["precision"][-1]*100:.1f}%  '
              f'R={data_map[key]["recall"][-1]*100:.1f}%')

    print('\nGenerating plots...')
    plot_metric(
        'map50', 'mAP@50 Over Training — All Models',
        'mAP@50', TARGET, 'map50_comparison.png', data_map)
    plot_metric(
        'precision', 'Precision Over Training — All Models',
        'Precision', None, 'precision_comparison.png', data_map)
    plot_metric(
        'recall', 'Recall Over Training — All Models',
        'Recall', None, 'recall_comparison.png', data_map)
    plot_summary_card(data_map)

    print(f'\nAll outputs saved to: {RESULTS}')
    print(f'\nYOLO26s final: mAP@50 = {data_map["YOLO26s"]["map50"][-1]*100:.1f}%  '
          f'(target ≥ {TARGET*100:.0f}%)  '
          f'[{"PASS" if data_map["YOLO26s"]["map50"][-1] >= TARGET else "FAIL"}]')


if __name__ == '__main__':
    main()
