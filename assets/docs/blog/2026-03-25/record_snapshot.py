#!/usr/bin/env python3
"""
Speed-Dx Benchmark — Snapshot Recorder
=======================================
Run this script at the END of each acceleration scenario run.
Press Enter when prompted to capture one snapshot of the current
pipeline state, then the script exits.

Run once per scenario run. Results accumulate in results/snapshot_NNN/.

USAGE
-----
  source ~/fsai_ws/install/setup.bash
  python3 testing/SpeedDx/record_snapshot.py
"""

import os
import sys
import shutil
import subprocess
import time
from pathlib import Path

BASE        = Path(__file__).parent
RESULTS     = BASE / 'results'
SPEED_LOG   = Path('/tmp/fsai_cone_speed_log.csv')
TIMEOUT_S   = 15


def next_snapshot_dir(results: Path) -> Path:
    """Auto-increment snapshot directory: snapshot_001, snapshot_002, ..."""
    existing = sorted(results.glob('snapshot_???'))
    if not existing:
        n = 1
    else:
        n = int(existing[-1].name.split('_')[1]) + 1
    return results / f'snapshot_{n:03d}'


def capture(out_dir: Path):
    out_dir.mkdir(parents=True, exist_ok=True)

    f_gt    = out_dir / 'gt_snapshot.txt'
    f_odom  = out_dir / 'odom_snapshot.txt'
    f_cones = out_dir / 'cones_snapshot.txt'

    print('Capturing from ROS 2 topics...', flush=True)
    p1 = subprocess.Popen(f'ros2 topic echo /carmaker/ObjectList --once > {f_gt}',   shell=True)
    p2 = subprocess.Popen(f'ros2 topic echo /carmaker/odom --once > {f_odom}',        shell=True)
    p3 = subprocess.Popen(f'ros2 topic echo /global_map/cones --once > {f_cones}',   shell=True)

    t0 = time.time()
    while p1.poll() is None or p2.poll() is None or p3.poll() is None:
        if time.time() - t0 > TIMEOUT_S:
            p1.terminate(); p2.terminate(); p3.terminate()
            print('\n[!] Timeout waiting for ROS 2 topics. Is the pipeline running?')
            sys.exit(1)
        time.sleep(0.4)
        sys.stdout.write('.')
        sys.stdout.flush()
    print()

    # Copy speed log
    if SPEED_LOG.exists():
        shutil.copy(SPEED_LOG, out_dir / 'speed_log.csv')
        print(f'Speed log copied ({SPEED_LOG.stat().st_size} bytes)')
    else:
        print(f'[!] Speed log not found at {SPEED_LOG} — did the local_mapper run?')


def main():
    snap_dir = next_snapshot_dir(RESULTS)
    n = int(snap_dir.name.split('_')[1])

    print(f'\n=== Speed-Dx Snapshot Recorder ===')
    print(f'Next snapshot: {snap_dir.name}')
    print()
    print('Drive the acceleration scenario, then come back here.')
    try:
        input(f'Press Enter to capture snapshot {n:03d}, or Ctrl+C to abort: ')
    except KeyboardInterrupt:
        print('\nAborted.')
        sys.exit(0)

    capture(snap_dir)

    print(f'\nSnapshot {n:03d} saved to: {snap_dir}')
    print('Contents:')
    for f in sorted(snap_dir.iterdir()):
        print(f'  {f.name}  ({f.stat().st_size} bytes)')
    print('\nDone. Run this script again for the next run.')


if __name__ == '__main__':
    main()
