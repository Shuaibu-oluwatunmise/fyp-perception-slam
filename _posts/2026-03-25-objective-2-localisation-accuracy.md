---
title: "Objective 2: Localisation Accuracy"
date: 2026-03-25 17:00:00 +0000
categories: [Week 10, Testing & Evaluation]
tags: [localisation, accuracy, fusion, benchmarking, evaluation, carmaker, odometry]
pin: false
image:
  path: /assets/img/thumbnails/41.png
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

From each matched pair, two errors are computed: **lateral error (dy)** — how far the mapped cone is from its true position side-to-side — and **longitudinal error (dx)** — how far it is along the direction of travel. The objective specifies positional error, so both matter.

## The Results

![Per-gate lateral and longitudinal errors with overhead cone map](/assets/img/blog/2026-03-25/per_cone_errors.png)
_Top: overhead map showing ground truth (white X) vs mapped cone positions. Middle: longitudinal error (dx) per gate. Bottom: lateral error (dy) per gate._

The lateral error across all 15 gates is **1.7 cm mean, ~3 cm max**. Against a 20 cm target, that's a 12x margin. The cone positions are accurate.

The longitudinal error is a different picture. It ranges from essentially zero up to around 1.5 m — well outside the 0.20 m target — and it appears to be speed-dependent. At a standstill the pipeline localises cones at exactly the correct position, and the error grows as the car moves faster. It's not trivial to dismiss either — on a straight the offset matters less, but on a turn it affects where the path planner thinks the corner apex is, which is safety-critical.

Detection rate is straightforward — all 30 cones across the 15 gates were detected and matched. **100%**, against a ≥95% target.

![Accuracy summary card](/assets/img/blog/2026-03-25/accuracy_summary_card.png)
_Summary card._

**Objective 2: PARTIAL PASS.** Detection rate: 100% ✓. Lateral error: 1.7 cm mean ✓. Longitudinal error: up to ~1.5 m under motion — exceeds the 0.20 m positional target ✗. The lateral accuracy is strong, but full positional accuracy as stated in the objective is not met under motion.

---

## Longitudinal Error Investigation

The speed-dependent pattern in the longitudinal error was clear enough to warrant a proper look. The question was whether the relationship between speed and forward error is actually linear — and if so, what that implies about the pipeline.

### How It Was Set Up

The pipeline needed modifications before any data could be collected. A `speed` field was added to the `Cone3D` message, the local mapper was updated to log the vehicle speed at the exact moment each cone was first added to the tracker, and the global mapper was updated to carry that field through to `/global_map/cones`. Speed at snapshot time is not the relevant quantity — it's the speed at the moment the cone was first seen that matters.

Two acceleration scenario runs were recorded at different speed profiles:

- **Run 1** — moderate profile, peak ~10.9 m/s (~39 km/h), 40 cones first detected while moving
- **Run 2** — higher profile, peak ~15.5 m/s (~56 km/h), 44 cones first detected while moving

A third run was captured but discarded — only 10 matched cone pairs, not enough to be statistically useful.

### Results

![Speed vs longitudinal error — scatter plot with linear fit](/assets/img/blog/2026-03-25/speed_dx_scatter.png)
_Each point is one matched cone pair. x-axis: vehicle speed at first detection. y-axis: dx error (positive = map behind GT, negative = map ahead of GT). Linear fit in red._

The linear fit across both runs:

```
dx = −0.130 · v + 0.181
```

- **Slope: −0.130 s** — implied end-to-end effective latency of ~130 ms
- **Static offset: +0.181 m** — residual error at zero speed
- **R² = 0.658** — moderately strong relationship given real-world variability

The negative slope means the map records cones *ahead* of their true position at higher speeds, not behind — the forward bias increases with speed.

![dx distribution by speed band](/assets/img/blog/2026-03-25/speed_dx_boxplot.png)
_Box plot of dx error grouped into 5 m/s bins. The trend is monotonic — each band shows greater forward bias than the one below it._

| Speed band | Mean dx |
|---|---|
| 0–5 m/s | +0.26 m |
| 5–10 m/s | −0.82 m |
| 10–15 m/s | −1.36 m |
| 15–20 m/s | −1.56 m |

At 10 m/s — a reasonable mid-range speed for an autocross or trackdrive scenario — the implied forward bias is around **1.3 m**.

![Per-run summary — mean dx vs mean speed](/assets/img/blog/2026-03-25/speed_dx_summary.png)
_Run 2 (higher speed) produced a larger mean absolute error than Run 1 — consistent with the speed-dx relationship, not an artefact of a single run._

### What Might Be Behind It

The sync test measured **80 ms** end-to-end from sensor input to `/cones`. The speed-dx investigation implies **~130 ms** of effective latency in the map. These are measuring different things: the sync test captures raw detection latency, while the map latency includes the local mapper's confirmation mechanism on top of that.

The local mapper requires 3 consecutive detections before promoting a cone to confirmed. At 30 Hz that's a minimum of ~67 ms. During that window the car has moved forward, and the EMA position averaging (α = 0.3) pulls the stored position toward later observations — which are from a slightly more forward vantage point. This could plausibly account for the gap between the two latency figures.

That said, this is the most consistent explanation available from the data — not a formally proven one. Odometry drift under acceleration, sensor timing offsets, and the coordinate frame transform chain are all factors that weren't isolated separately. The R² of 0.658 leaves room for other contributors.

One potential mitigation would be to apply a predictive correction using the measured latency constant when a cone is first confirmed — back-calculating its position using the odom history from ~130 ms earlier. That would reduce the forward bias to near zero at any speed. It's a future work item.

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

**[Longitudinal Error Recorder](/fyp-perception-slam/assets/docs/blog/2026-03-25/record_snapshot.py)** — `record_snapshot.py`
_Captures a consistent end-of-run snapshot (ground truth, odom, global map, speed log) for one acceleration scenario run_

**[Longitudinal Error Analysis](/fyp-perception-slam/assets/docs/blog/2026-03-25/analyse_snapshots.py)** — `analyse_snapshots.py`
_Loads all recorded snapshots, matches cone pairs, looks up speed at first detection, fits linear regression, generates scatter/boxplot/summary plots_
