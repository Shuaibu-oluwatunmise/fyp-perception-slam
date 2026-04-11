---
title: "Final Submission: The System That Shipped"
date: 2026-04-07 00:00:00 +0000
categories: [Final Submission]
tags: [fyp, perception, slam, evaluation, final, formula-student, ros2, yolo, sensor-fusion]
pin: true
featured: true
image:
  path: /assets/img/thumbnails/44.jpeg
  alt: Final submission — simulation running live on the lab display
  header: false
---

Twelve weeks. Five nodes. One system that actually runs.

The video below is a full demonstration of the complete project — cone detection, sensor fusion, localisation, and mapping running end-to-end in simulation on the target hardware.

{% include embed/youtube.html id='sJ6NbEHHeeU' %}
_Full end-to-end pipeline running live in IPG CarMaker — camera detection, LiDAR fusion, and landmark mapping on the Jetson AGX Orin._

## What Was Built

What I built is a perception and SLAM pipeline for Formula Student AI — five nodes, running on a Jetson AGX Orin, connected to IPG CarMaker over a ROS 2 bridge. Camera detects cones. LiDAR detects cones. A fusion engine matches them using the Hungarian algorithm. A landmark mapper accumulates confirmed detections into a persistent cone map as the car drives.

The full stack:

| Node | Role |
|---|---|
| 1a / 1b | Camera and LiDAR publishers — live sensor data over the CarMaker bridge |
| 2a / 2b | YOLOv26s cone detector (camera) and DBSCAN cluster extractor (LiDAR) |
| 3 | Late sensor fusion engine — Hungarian matching of camera and LiDAR detections |
| 4 | Frame transformer — sensor-frame detections into the vehicle coordinate frame |
| 5 | Landmark mapper — incremental cone map from confirmed fused detections |

## The Results

| Objective | Metric | Target | Result | Status |
|---|---|---|---|---|
| 1a — Real-Time Vision | mAP@50 | ≥85% | 87.8% | PASS |
| 1b — Real-Time Vision | FPS | ≥30 | 35.1 | PASS |
| 2a — Localisation | Detection rate | ≥95% | 100% | PASS |
| 2b — Localisation | Lateral error (dy) | ≤0.20 m | 1.7 cm mean | PASS |
| 2c — Localisation | Longitudinal error (dx) | ≤0.20 m | up to ~1.5 m | FAIL |
| 3 — Fusion Range | Max range | ≥20 m | 25.1 m | PASS |
| 4 — Latency | End-to-end p95 | ≤100 ms | 99.0 ms | PASS (qualified) |

Five pass, one fails. The lateral accuracy is the headline — 1.7 cm mean against a 20 cm target is a 12x margin. The fusion architecture, projecting LiDAR into image space and matching against YOLO bounding boxes, does exactly what it was designed to do.

The longitudinal failure is the honest result. Up to 1.5 m of forward bias under motion, scaling linearly with speed, pointing to a timing issue in the mapper's confirmation mechanism compounding raw sensor latency. It's documented, understood in outline, and not papered over. The [full investigation is here](/fyp-perception-slam/posts/objective-2-localisation-accuracy/).

## Reflection

Some of this went further than planned. Some of it didn't get there.

The late fusion engine wasn't in the original plan. The first version was geometrically correct — it just fell apart under motion. I rewrote it twice. The image-space LiDAR projection approach with Hungarian matching is what ended up running, and honestly the lateral accuracy numbers are a direct result of getting that right. I don't regret the time it took.

I ran everything on the Jetson throughout rather than a development machine. That was deliberate. 35 FPS and 99 ms p95 on embedded hardware means something different to the same numbers on a laptop. That constraint was worth keeping.

The longitudinal error is the one thing I'd go back and fix. The investigation shows it scales linearly with speed — implied ~130 ms effective latency compounding through the pipeline. The cause is probably tractable. It just didn't make it within the twelve weeks.

## What Comes Next

Given more time, this is where I'd take it:

1. **Real hardware deployment** — taking the pipeline from CarMaker simulation onto a physical Formula Student vehicle with real sensors, real noise, and real track conditions
2. **Fix longitudinal error** — isolate whether it's the mapper confirmation delay or the raw pipeline latency, and address it directly; TensorRT quantisation on YOLO would also push the p95 tail further from the 100 ms limit
3. **Loop closure for the mapper** — the current mapper grows incrementally but does not close loops; a constraint-based approach would dramatically improve consistency over a full lap
4. **Larger training dataset** — 87.8% mAP@50 is solid, but the FSOCO dataset has limited diversity in lighting and track layout; better real-world generalisation requires more varied training data
5. **Path planning integration** — the perception stack produces a cone map; feeding it into a path planner to generate a racing line is the natural next step to complete the autonomy loop
6. **LiDAR intensity-based cone colour classification** — LiDAR returns include per-point intensity values that vary with surface reflectance; yellow and blue cones have measurably different reflectance characteristics, which could enable colour classification from LiDAR alone — particularly valuable in low-light or high-glare conditions where camera-based colour detection degrades

## The Journey

Forty-three posts. These are the ones that mark the actual turning points:

| | Post |
|---|---|
| Start | [Week 1: Project Selection](/fyp-perception-slam/posts/week-1-project-selection/) |
| The blueprint | [What Am I Actually Building?](/fyp-perception-slam/posts/perception-slam-system-explained/) |
| Phase 1 done | [Building Node 3: Phase 1 Complete](/fyp-perception-slam/posts/building-node-3/) |
| The decision | [The Big Pivot to Linux](/fyp-perception-slam/posts/the-big-pivot-to-linux/) |
| First live run | [The Bridge: Connecting the Pipeline to CarMaker](/fyp-perception-slam/posts/the-bridge-connecting-the-pipeline-to-carmaker/) |
| The rewrite | [The Fusion Architecture: Rewriting Node 3 Twice](/fyp-perception-slam/posts/the-fusion-architecture-rewriting-node-3-twice-to-get-it-right/) |
| The algorithm | [The Late Sensor Fusion Engine and the Hungarian Matrix](/fyp-perception-slam/posts/the-late-sensor-fusion-engine-and-the-hungarian-matrix/) |
| SLAM | [The Mapper That Actually Shipped](/fyp-perception-slam/posts/the-mapper-that-shipped/) |
| The verdict | [Does It Meet the Objectives?](/fyp-perception-slam/posts/does-it-meet-the-objectives/) |
