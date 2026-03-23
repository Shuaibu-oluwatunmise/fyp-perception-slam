---
title: "Objective 2: Localisation Accuracy"
date: 2026-03-25 17:00:00 +0000
categories: [Week 10, Testing & Evaluation]
tags: [localisation, accuracy, fusion, benchmarking, evaluation, carmaker, odometry]
pin: false
image:
  path: /assets/img/thumbnails/38.png
  alt: Localisation Accuracy Results
  header: false
---

This was the one I was most uncertain about going in. The detection side — mAP, FPS — those are well-understood metrics with well-understood benchmarks. Localisation accuracy is trickier. It depends on the full pipeline running correctly: YOLO detecting the cone, LiDAR confirming the position, fusion resolving them together, the mapper accumulating detections into a consistent global map. Any one of those stages going wrong shows up here.

Objective 2 has two targets: ≥95% cone detection rate, and ≤0.20 m positional error at 20–30 m range.

## How It Was Measured

The benchmark runs the full perception pipeline in the CarMaker Acceleration scenario — the same straight track of 15 cone gate pairs used throughout testing. The car drives through the gates while the perception pipeline runs live. Once the car has passed through enough of the track, the script takes a simultaneous snapshot of three ROS2 topics:

- `/carmaker/ObjectList` — ground truth cone positions from the simulation
- `/carmaker/odom` — the car's current position and orientation
- `/global_map/cones` — the mapped cone positions output by the perception pipeline

The ground truth positions come in the vehicle body frame, so they're transformed into the odometry frame using the car's pose at snapshot time. The mapped cones are already in the odometry frame. Euclidean nearest-neighbour matching then pairs each mapped cone to its closest ground truth cone, with a 3.0 m search threshold. The first two gates are skipped — the car spawns near them and the pipeline hasn't fully initialised at that point.

From each matched pair, two errors are computed: **lateral error (dy)** — how far the mapped cone is from its true position side-to-side — and **longitudinal error (dx)** — how far it is along the direction of travel.

The objective specifies positional error, and in the context of a car navigating between cone gates, lateral error is what actually matters. A cone that's slightly too far or close along the track direction is far less dangerous than one that's reported on the wrong side.

## The Results

![Per-gate lateral and longitudinal errors with overhead cone map](/assets/img/blog/2026-03-25/per_cone_errors.png)
_Top: overhead map showing ground truth (white X) vs mapped cone positions. Middle: longitudinal error (dx) per gate. Bottom: lateral error (dy) per gate._

The lateral error across all 15 gates is **1.7 cm mean, ~3 cm max**. Against a 20 cm target, that's a 12x margin. The cone positions are accurate.

The longitudinal error is a different picture. It ranges from essentially zero up to around 1.5 m, and it appears to be speed-dependent — at a standstill the pipeline localises cones at exactly the correct position, and the error grows as the car moves faster. This relationship hasn't been formally characterised since it's outside the project scope, but the pattern is consistent enough to point at vehicle speed as the main factor rather than a perception failure. The perception system is placing cones correctly relative to where it thinks the car is — the offset comes from somewhere in the pose estimation under motion, not from the detection itself.

Detection rate is straightforward — all 30 cones across the 15 gates were detected and matched. **100%**, against a ≥95% target.

![Accuracy summary card](/assets/img/blog/2026-03-25/accuracy_summary_card.png)
_Summary card. Both metrics pass._

**Objective 2: PASS — 100% detection rate, 1.7 cm mean lateral error against ≤20 cm target.** Longitudinal error of up to ~1.5 m was observed under motion and is noted as a speed-dependent effect for future investigation.

---

Tomorrow covers Objective 3 and 4 — fusion range and end-to-end latency. Two results, one post.

---

## Resources

**[Accuracy Benchmark Script](/fyp-perception-slam/assets/docs/blog/2026-03-25/obj2_accuracy_bench.py)** — `obj2_accuracy_bench.py`
_Captures live ROS2 snapshots, matches mapped cones to ground truth, computes lateral and longitudinal errors_

**[Mapped Cones Snapshot](/fyp-perception-slam/assets/docs/blog/2026-03-25/cones_snapshot.txt)** — `cones_snapshot.txt`
_Raw `/global_map/cones` topic output at time of measurement_

**[Ground Truth Snapshot](/fyp-perception-slam/assets/docs/blog/2026-03-25/gt_snapshot.txt)** — `gt_snapshot.txt`
_Raw `/carmaker/ObjectList` topic output — simulation ground truth cone positions_

**[Odometry Snapshot](/fyp-perception-slam/assets/docs/blog/2026-03-25/odom_snapshot.txt)** — `odom_snapshot.txt`
_Car pose at snapshot time, used to transform ground truth into odometry frame_
