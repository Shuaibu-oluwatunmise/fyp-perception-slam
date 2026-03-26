---
title: "Objectives 3 and 4: Range and Latency"
date: 2026-03-26 00:00:00 +0000
categories: [Week 10, Testing & Evaluation]
tags: [sensor-fusion, lidar, camera, latency, benchmarking, evaluation, ros2]
pin: false
image:
  path: /assets/img/thumbnails/42.png
  alt: Range and Latency Results
  header: false
---

Two objectives left before the full picture. Objective 3 is about how far the fusion pipeline can see. Objective 4 is about how fast it processes what it sees. Different things, but both matter for a system that has to react to a track it's discovering in real time.

---

## Objective 3 — Fusion Range

The target is ≥20 m. The question is whether the fused output actually preserves the hardware range limits of the sensors — or whether the fusion step itself introduces drops.

### How It Was Measured

Four ROS2 topics are snapshotted simultaneously:

- `/carmaker/ObjectList` — ground truth cone positions from the simulation
- `/detections` — YOLO 2D bounding boxes from the camera
- `/detections/lidar` — raw DBSCAN clusters from the LiDAR
- `/cones` — the fused output from the localizer

For each modality, the script finds the furthest cone successfully resolved. One frame offset needed correcting: the LiDAR publishes positions in `Lidar_F` (relative to the front bumper), while the localizer publishes in `Fr1A` (relative to the rear axle). There's a 2.9 m difference between those two reference points, so the fused output is normalised before comparing against LiDAR range.

Camera range is estimated differently — since YOLO outputs 2D boxes with no depth, range can only be bounded. If the camera detected N cones, the range lies between the Nth and the (N+1)th ground truth cone distance.

### The Results

![Range benchmark — bar chart comparing camera, LiDAR and fused output](/assets/img/blog/2026-03-26/range_bars.png)
_Detection range per modality. All three exceed the 20 m target. Fused range matches LiDAR — fusion doesn't drop detections._

![Range summary card](/assets/img/blog/2026-03-26/range_summary_card.png)
_Summary card. All modalities pass._

LiDAR maxes out at **25.1 m**. The fused output matches it exactly — the fusion step doesn't lose any detections, which is what it should do. Camera range sits in the 24–29 m interval estimate, consistent with LiDAR coverage.

The fusion bottleneck is LiDAR, as expected. LiDAR point density drops off sharply with distance — at 25 m a cone is represented by just 3 points, which is the minimum DBSCAN accepts as a valid cluster. Beyond that, clusters don't form and there's nothing for the fusion engine to work with. The camera can still see cones at that range but without a LiDAR match the detection is dropped — by design, since there's no depth source reliable enough to assign a 3D position.

The LiDAR also has a near-field limit worth mentioning. The detector filters out any points closer than 0.2 m, so the effective detection window is roughly **0.2 m to 25.1 m**, not zero to 25.1 m. In practice for the Acceleration scenario this doesn't cause issues since the car is moving forward and cones are always at distance, but it's a real constraint for tighter scenarios like the Skidpad where cones can be nearly adjacent to the vehicle.

**Objective 3: PASS — 25.1 m fused range against a ≥20 m target.**

---

## Objective 4 — End-to-End Latency

The target is ≤100 ms end-to-end. This is the wall-clock time from the earliest sensor input arriving to the fused cone being published — the full pipeline delay a navigation stack would experience.

### How It Was Measured

The benchmark node subscribes to all five relevant topics and timestamps every message arrival:

- Raw camera frame in (`/camera/image_raw`)
- Raw LiDAR scan in (`/lidar/pointcloud`)
- YOLO detection out (`/detections`)
- DBSCAN cluster out (`/detections/lidar`)
- Fused cone out (`/cones`)

Per-stage latency is the difference between an output timestamp and the nearest prior input timestamp. End-to-end latency uses `min(camera_arrival, lidar_arrival)` as the cycle start — the earliest moment any sensor data for that cycle was available. Using `max()` instead would give physically impossible results below YOLO's own inference time.

Three trials of 100 frames run with a 5-second cooldown between them to let the pipeline settle. One ROS2 quirk worth noting: `create_timer` is a repeating timer, not one-shot. The cooldown timer has to be explicitly cancelled inside its own callback, otherwise it fires again and resets the trial indefinitely.

### The Results

![End-to-end latency analysis — violin, bar, and trace plots](/assets/img/blog/2026-03-26/latency_analysis.png)
_Top: per-node latency distribution. Middle: pipeline breakdown with run-to-run error bars. Bottom: per-frame total latency trace across all 3 trials._

The numbers:

| Stage | Mean | p95 |
|---|---|---|
| YOLO26s inference | 61.1 ms | 76.4 ms |
| DBSCAN clustering | 43.4 ms | 197.3 ms |
| Fusion | 13.4 ms | 26.9 ms |
| **End-to-end** | **80.0 ms** | **99.0 ms** |

YOLO is the consistent bottleneck at 61.1 ms mean. DBSCAN runs in parallel at 43.4 ms — since it's always faster than YOLO, it never extends the critical path. Fusion adds 13.4 ms on top of whichever finishes last.

The DBSCAN p95 of 197 ms looks alarming but it doesn't matter much in practice. That spike only affects total latency on frames where DBSCAN takes longer than YOLO — which is rare. When it does happen, total latency spikes above the mean but the 3-trial trace shows this is infrequent.

The p95 end-to-end is **99.0 ms**. p95 means 95% of frames complete within that time — so technically the target is met, but 5% of frames are taking longer than 99 ms and some of those will exceed 100 ms. It's a pass on the metric as defined, but it's not a clean one. The mean of 80 ms has 20 ms headroom, but the tail is close to the edge. Worth noting that none of this uses TensorRT — YOLO26s runs on raw CUDA. TensorRT would bring inference down to roughly 30–40 ms on the Jetson, which would push the p95 well clear of the target.

Run-to-run standard deviation is ±0.9 ms — the system is thermally stable across separate trials, no drift between runs.

**Objective 4: PASS (qualified) — 80.0 ms mean, 99.0 ms p95 against a ≤100 ms target. 5% of frames exceed 99 ms and some will breach 100 ms.**

---

Tomorrow is the full picture — all five benchmarks together.

---

## Resources

**[Range Benchmark Script](/fyp-perception-slam/assets/docs/blog/2026-03-26/obj3_range_bench.py)** — `obj3_range_bench.py`
_Captures four topic snapshots simultaneously, normalises coordinate frames, computes max range per modality_

**[Latency Benchmark Script](/fyp-perception-slam/assets/docs/blog/2026-03-26/sync_test.py)** — `sync_test.py`
_Timestamps all pipeline stages across 3 trials, computes per-stage mean, p95 and run-to-run std_

**[Fused Cones Snapshot](/fyp-perception-slam/assets/docs/blog/2026-03-26/fused_snapshot.txt)** — `fused_snapshot.txt`
_Raw `/cones` topic output at time of range measurement_

**[LiDAR Clusters Snapshot](/fyp-perception-slam/assets/docs/blog/2026-03-26/lidar_snapshot.txt)** — `lidar_snapshot.txt`
_Raw `/detections/lidar` topic output — DBSCAN cluster positions_

**[YOLO Detections Snapshot](/fyp-perception-slam/assets/docs/blog/2026-03-26/detections_snapshot.txt)** — `detections_snapshot.txt`
_Raw `/detections` topic output — YOLO 2D bounding boxes_

**[Ground Truth Snapshot](/fyp-perception-slam/assets/docs/blog/2026-03-26/range_gt_snapshot.txt)** — `range_gt_snapshot.txt`
_Raw `/carmaker/ObjectList` topic output — simulation ground truth cone positions_
