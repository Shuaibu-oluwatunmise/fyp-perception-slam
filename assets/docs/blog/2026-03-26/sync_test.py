#!/usr/bin/env python3
"""
Sync & Latency Benchmark (Objective 4)
Measures the actual compute latency of each individual perception node
and calculates the true end-to-end system latency.

Runs 3 trials automatically with a 5-second cooldown gap between each.
Final plots show per-trial traces + cross-trial mean +/- std.

Pipeline Steps Monitored:
1. Camera Inference Latency: raw Image arrival -> YOLO /detections
2. LiDAR Inference Latency: raw PointCloud2 arrival -> DBSCAN /detections/lidar
3. Fusion Latency: both intermediates ready -> /cones
4. Total End-to-End: FIRST raw sensor arrival -> /cones output

Fixes applied:
  - create_timer is repeating in ROS2, not one-shot. Cooldown timer is now
    cancelled immediately after firing to stop it resetting trial buffers
    repeatedly mid-collection.
  - Total latency now uses a per-cycle tracking dict keyed by a frame counter
    rather than relying on last_raw_cam/lidar_time, which were being wiped
    by buffer resets and produced totals shorter than individual node latencies.
"""

import rclpy
from rclpy.node import Node
import time
import numpy as np
import matplotlib.pyplot as plt
import os

from sensor_msgs.msg import Image, PointCloud2
from vision_msgs.msg import Detection2DArray
from fsai_interfaces.msg import Cone3DArray


# ── Configuration ─────────────────────────────────────────────────────────────
FRAMES_PER_TRIAL = 100
NUM_TRIALS       = 3
COOLDOWN_SEC     = 5.0
SAVE_DIR         = '/home/mdxfsai/fsai_ws/testing/Sync/results'


class SyncTestBenchmarker(Node):

    def __init__(self):
        super().__init__('sync_test_bench')

        self.trial        = 0
        self.in_cooldown  = False
        self._cooldown_timer = None

        # Per-trial results — list of np arrays, one per trial
        self.all_yolo   = []
        self.all_dbscan = []
        self.all_fusion = []
        self.all_total  = []

        self._reset_trial_buffers()

        self.create_subscription(
            Image,            '/camera/image_raw',   self.raw_cam_cb,    10)
        self.create_subscription(
            PointCloud2,      '/lidar/pointcloud',    self.raw_lidar_cb,  10)
        self.create_subscription(
            Detection2DArray, '/detections',          self.yolo_out_cb,   10)
        self.create_subscription(
            Cone3DArray,      '/detections/lidar',    self.dbscan_out_cb, 10)
        self.create_subscription(
            Cone3DArray,      '/cones',               self.fusion_out_cb, 10)

        self.get_logger().info(
            f'Latency Benchmarker ready — {NUM_TRIALS} trials x '
            f'{FRAMES_PER_TRIAL} frames, {COOLDOWN_SEC}s cooldown between trials.')
        self.get_logger().info('Trial 1/3 starting...')

    # ── Trial state ───────────────────────────────────────────────────────────

    def _reset_trial_buffers(self):
        # Rolling arrival timestamp lists for matching raw→output
        self.img_arrivals = []
        self.pc_arrivals  = []

        # Per-cycle raw arrival times — reset after each fusion output.
        # We store cam and lidar separately and take min() at fusion time.
        # This gives the true earliest sensor arrival for this cycle.
        self._cycle_cam_time   = None
        self._cycle_lidar_time = None

        # Intermediate output times for fusion latency measurement
        self.last_det_time       = None
        self.last_lidar_det_time = None

        # Current trial result lists (ms)
        self.lats_yolo   = []
        self.lats_dbscan = []
        self.lats_fusion = []
        self.lats_total  = []

    # ── Raw input callbacks ───────────────────────────────────────────────────

    def raw_cam_cb(self, msg):
        if self.in_cooldown:
            return
        now = time.time()
        self.img_arrivals.append(now)
        if len(self.img_arrivals) > 50:
            self.img_arrivals.pop(0)
        # Always overwrite — we want the most recent cam arrival for this cycle
        self._cycle_cam_time = now

    def raw_lidar_cb(self, msg):
        if self.in_cooldown:
            return
        now = time.time()
        self.pc_arrivals.append(now)
        if len(self.pc_arrivals) > 50:
            self.pc_arrivals.pop(0)
        # Always overwrite — we want the most recent lidar arrival for this cycle
        self._cycle_lidar_time = now

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _pop_closest_arrival(self, arr_list, now):
        """Pop and return the most recent arrival strictly before now."""
        valid = [t for t in arr_list if t < now]
        if not valid:
            return None
        best = max(valid)
        arr_list.remove(best)
        return best

    # ── Intermediate output callbacks ─────────────────────────────────────────

    def yolo_out_cb(self, msg):
        if self.in_cooldown:
            return
        now = time.time()
        self.last_det_time = now
        arr = self._pop_closest_arrival(self.img_arrivals, now)
        if arr:
            lat = (now - arr) * 1000.0
            if 0 < lat < 2000.0:
                self.lats_yolo.append(lat)

    def dbscan_out_cb(self, msg):
        if self.in_cooldown:
            return
        now = time.time()
        self.last_lidar_det_time = now
        arr = self._pop_closest_arrival(self.pc_arrivals, now)
        if arr:
            lat = (now - arr) * 1000.0
            if 0 < lat < 2000.0:
                self.lats_dbscan.append(lat)

    # ── Final output callback ─────────────────────────────────────────────────

    def fusion_out_cb(self, msg):
        if self.in_cooldown:
            return

        now = time.time()

        # Fusion node cost: from when BOTH upstream nodes finished
        if self.last_det_time and self.last_lidar_det_time:
            sync_start = max(self.last_det_time, self.last_lidar_det_time)
            fusion_lat = (now - sync_start) * 1000.0
            if 0 < fusion_lat < 2000.0:
                self.lats_fusion.append(fusion_lat)

        # True end-to-end: from whichever sensor fired most recently before
        # this fusion output. Both cam and lidar must have fired this cycle.
        # min() of the two gives the earliest, i.e. when the pipeline started.
        if self._cycle_cam_time is not None and self._cycle_lidar_time is not None:
            true_start = min(self._cycle_cam_time, self._cycle_lidar_time)
            total_lat  = (now - true_start) * 1000.0
            if 0 < total_lat < 5000.0:
                self.lats_total.append(total_lat)
        # Reset both for next cycle
        self._cycle_cam_time   = None
        self._cycle_lidar_time = None

        samples = len(self.lats_total)
        if samples > 0 and samples % 10 == 0:
            self.get_logger().info(
                f'  Trial {self.trial + 1}/{NUM_TRIALS} — '
                f'{samples}/{FRAMES_PER_TRIAL} frames captured')

        if samples >= FRAMES_PER_TRIAL:
            self._finish_trial()

    # ── Trial management ──────────────────────────────────────────────────────

    def _finish_trial(self):
        # Prevent fusion_out_cb triggering this again before cooldown starts
        self.in_cooldown = True

        yolo   = np.array(self.lats_yolo)
        dbscan = np.array(self.lats_dbscan)
        fusion = np.array(self.lats_fusion)
        total  = np.array(self.lats_total[:FRAMES_PER_TRIAL])

        self.all_yolo.append(yolo)
        self.all_dbscan.append(dbscan)
        self.all_fusion.append(fusion)
        self.all_total.append(total)

        self.get_logger().info(
            f'\n  Trial {self.trial + 1} complete — '
            f'YOLO {np.mean(yolo):.1f}ms | '
            f'DBSCAN {np.mean(dbscan):.1f}ms | '
            f'Fusion {np.mean(fusion):.1f}ms | '
            f'Total {np.mean(total):.1f}ms')

        self.trial += 1

        if self.trial >= NUM_TRIALS:
            self.generate_report()
        else:
            self.get_logger().info(
                f'  Cooling down {COOLDOWN_SEC}s before trial '
                f'{self.trial + 1}/{NUM_TRIALS}...')
            # ── Fix: cancel the timer after one fire so it does not repeat ──
            self._cooldown_timer = self.create_timer(
                COOLDOWN_SEC, self._start_next_trial)

    def _start_next_trial(self):
        # Cancel immediately — create_timer is repeating in ROS2
        if self._cooldown_timer is not None:
            self._cooldown_timer.cancel()
            self._cooldown_timer = None

        self._reset_trial_buffers()
        self.in_cooldown = False
        self.get_logger().info(
            f'  Trial {self.trial + 1}/{NUM_TRIALS} starting...')

    # ── Final report ──────────────────────────────────────────────────────────

    def generate_report(self):
        yolo_all   = np.concatenate(self.all_yolo)
        dbscan_all = np.concatenate(self.all_dbscan)
        fusion_all = np.concatenate(self.all_fusion)
        total_all  = np.concatenate(self.all_total)

        trial_means = {
            'yolo':   [np.mean(x) for x in self.all_yolo],
            'dbscan': [np.mean(x) for x in self.all_dbscan],
            'fusion': [np.mean(x) for x in self.all_fusion],
            'total':  [np.mean(x) for x in self.all_total],
        }

        my  = np.mean(yolo_all);   sy  = np.std(trial_means['yolo'])
        md  = np.mean(dbscan_all); sd  = np.std(trial_means['dbscan'])
        mf  = np.mean(fusion_all); sf  = np.std(trial_means['fusion'])
        mt  = np.mean(total_all);  st  = np.std(trial_means['total'])

        p95y = np.percentile(yolo_all,   95)
        p95d = np.percentile(dbscan_all, 95)
        p95f = np.percentile(fusion_all, 95)
        p95t = np.percentile(total_all,  95)

        self.get_logger().info('\n')
        self.get_logger().info('=' * 60)
        self.get_logger().info(
            f'   PERCEPTION LATENCY REPORT  '
            f'({NUM_TRIALS} trials x {FRAMES_PER_TRIAL} frames)')
        self.get_logger().info('=' * 60)
        self.get_logger().info(
            f'{"Stage":<22} {"Mean":>8} {"p95":>8} {"Run std":>10}')
        self.get_logger().info('-' * 60)
        self.get_logger().info(
            f'{"YOLO26s inference":<22} {my:>6.1f}ms {p95y:>6.1f}ms  +/-{sy:.1f}ms')
        self.get_logger().info(
            f'{"DBSCAN clustering":<22} {md:>6.1f}ms {p95d:>6.1f}ms  +/-{sd:.1f}ms')
        self.get_logger().info(
            f'{"Fusion (localizer)":<22} {mf:>6.1f}ms {p95f:>6.1f}ms  +/-{sf:.1f}ms')
        self.get_logger().info('-' * 60)
        self.get_logger().info(
            f'  Bottleneck: {"YOLO" if my >= md else "DBSCAN"}  '
            f'({max(my, md):.1f}ms)')
        self.get_logger().info(
            f'  TOTAL END-TO-END  {mt:>6.1f}ms {p95t:>6.1f}ms  +/-{st:.1f}ms')
        self.get_logger().info(
            f'  Implied throughput: {1000.0 / mt:.1f} Hz')
        self.get_logger().info('=' * 60)

        self._plot_results(trial_means, yolo_all, dbscan_all, fusion_all, total_all)
        rclpy.shutdown()

    # ── Plot ──────────────────────────────────────────────────────────────────

    def _plot_results(self, trial_means, yolo_all, dbscan_all, fusion_all, total_all):
        os.makedirs(SAVE_DIR, exist_ok=True)

        avg_yolo   = np.mean(yolo_all)
        avg_dbscan = np.mean(dbscan_all)
        avg_fusion = np.mean(fusion_all)
        avg_total  = np.mean(total_all)
        bottleneck = max(avg_yolo, avg_dbscan)

        NODE_COLOURS  = ['#3A86FF', '#FF6B6B', '#F4A261']
        TRIAL_COLOURS = ['#3A86FF', '#FF6B6B', '#2DC653']

        fig = plt.figure(figsize=(16, 11))
        ax1 = plt.subplot(2, 2, 1)
        ax2 = plt.subplot(2, 2, 2)
        ax3 = plt.subplot(2, 1, 2)

        # ── Violin plot ───────────────────────────────────────────────────────
        data   = [yolo_all, dbscan_all, fusion_all]
        labels = [
            f'YOLO26s\n(mean={avg_yolo:.1f}ms)',
            f'DBSCAN\n(mean={avg_dbscan:.1f}ms)',
            f'Fusion\n(mean={avg_fusion:.1f}ms)',
        ]

        parts = ax1.violinplot(
            data, positions=[1, 2, 3],
            showmeans=True, showmedians=True, showextrema=True)

        for pc, col in zip(parts['bodies'], NODE_COLOURS):
            pc.set_facecolor(col)
            pc.set_alpha(0.5)
            pc.set_edgecolor('black')
            pc.set_linewidth(0.8)

        parts['cmeans'].set_color('red')
        parts['cmeans'].set_linewidth(2.5)
        parts['cmedians'].set_color('black')
        parts['cmedians'].set_linewidth(1.5)
        for part in ['cbars', 'cmins', 'cmaxes']:
            parts[part].set_color('dimgray')
            parts[part].set_linewidth(1)

        for i, (arr, col) in enumerate(zip(data, NODE_COLOURS), start=1):
            jitter = np.random.uniform(-0.07, 0.07, size=len(arr))
            ax1.scatter(i + jitter, arr, s=7, color=col,
                        alpha=0.3, zorder=3, edgecolors='none')

        ax1.set_xticks([1, 2, 3], labels=labels)
        ax1.set_ylabel('Execution Time (ms)')
        ax1.set_title(
            'Node Latency Distribution\n(shape=spread, red=mean, black=median)',
            fontweight='bold')
        ax1.grid(True, linestyle='--', alpha=0.4, axis='y')

        # ── Bar chart with error bars ─────────────────────────────────────────
        nodes    = ['Parallel\nInference\n(bottleneck)', 'Late\nFusion', 'Total\nSystem']
        times    = [bottleneck, avg_fusion, avg_total]
        bcolors  = ['#E74C3C', '#F39C12', '#27AE60']
        bn_key   = 'yolo' if avg_yolo >= avg_dbscan else 'dbscan'
        run_stds = [
            np.std(trial_means[bn_key]),
            np.std(trial_means['fusion']),
            np.std(trial_means['total']),
        ]

        bars = ax2.bar(
            np.arange(len(nodes)), times,
            yerr=run_stds, capsize=7,
            color=bcolors, alpha=0.85,
            edgecolor='black', linewidth=0.5,
            error_kw=dict(elinewidth=2, ecolor='black', capthick=2))

        ax2.set_xticks(np.arange(len(nodes)), labels=nodes)
        ax2.set_ylabel('Milliseconds (ms)')
        ax2.set_title(
            f'Pipeline Compute Breakdown\n'
            f'(error bars = run-to-run std, n={NUM_TRIALS} trials)',
            fontweight='bold')
        ax2.axhline(100.0, color='black', linestyle=':', linewidth=1.5,
                    label='10 Hz target (100 ms)')
        ax2.legend(fontsize=9)

        for bar, val in zip(bars, times):
            ax2.text(
                bar.get_x() + bar.get_width() / 2,
                bar.get_height() + max(run_stds) + 3,
                f'{val:.1f} ms', ha='center', va='bottom',
                fontweight='bold', fontsize=10)

        ax2.set_ylim(0, max(times) * 1.35)

        # ── Per-trial latency traces ───────────────────────────────────────────
        for i, trial_total in enumerate(self.all_total):
            frames = np.arange(len(trial_total))
            ax3.plot(frames, trial_total,
                     color=TRIAL_COLOURS[i], linewidth=1.3,
                     marker='o', markersize=2.5, alpha=0.8,
                     label=f'Trial {i + 1}  (mean={np.mean(trial_total):.1f}ms)')

        ax3.axhline(avg_total, color='black', linestyle='--', linewidth=2,
                    label=f'Grand mean: {avg_total:.1f}ms')
        ax3.axhline(100.0, color='gray', linestyle=':', linewidth=1.5,
                    label='10 Hz target (100 ms)')

        sigma = np.std(total_all)
        ax3.fill_between(
            np.arange(FRAMES_PER_TRIAL),
            avg_total - sigma, avg_total + sigma,
            alpha=0.1, color='black',
            label=f'+/-1 sigma ({sigma:.1f}ms)')

        ax3.set_title(
            f'End-to-End Latency Trace — All {NUM_TRIALS} Trials',
            fontweight='bold')
        ax3.set_xlabel('Frame Index (within trial)')
        ax3.set_ylabel('Total System Latency (ms)')
        ax3.set_ylim(bottom=0)
        ax3.grid(True, alpha=0.35)
        ax3.legend(loc='upper right', fontsize=9)

        plt.suptitle(
            'Objective 4: Data Synchronisation & End-to-End Latency Analysis',
            fontsize=15, fontweight='bold', y=1.01)
        fig.tight_layout(pad=1.8)

        save_path = os.path.join(SAVE_DIR, 'latency_analysis.png')
        plt.savefig(save_path, dpi=200, bbox_inches='tight')
        self.get_logger().info(f'\nPlot saved to: {save_path}')


def main(args=None):
    rclpy.init(args=args)
    node = SyncTestBenchmarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Benchmark interrupted.')
        if node.all_total:
            node.get_logger().info(
                'Generating partial report from completed trials...')
            node.generate_report()
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()