---
title: "How Do You Know If It Works?"
date: 2026-03-23 17:00:00 +0000
categories: [Week 10, Testing & Evaluation]
tags: [testing, benchmarking, methodology, perception, ros2, carmaker, evaluation]
pin: false
image:
  path: /assets/img/thumbnails/39.png
  alt: Testing Methodology
  header: false
---

Up until now, the evidence that the system works has mostly been visual. A cone appears in RViz at roughly the right place. The colour label looks correct. The fused markers stay stable on a Skidpad run. That's useful for debugging, but it's not a measurement — it's just watching things happen and deciding they look right.

The objectives I set for this project don't say "looks right." They say ≥85% mAP@50, ≥30 FPS, ≤0.20 m positional error, ≥95% detection rate, ≤100 ms end-to-end latency. Those are numbers. So this week is about finding out if the system actually produces them.

## The Problem with Manual Observation

The tempting approach is to run the simulation and watch. If the cones show up in the right places most of the time, you call it good. But "most of the time" is doing a lot of work in that sentence, and it's not reproducible. Someone else running the same scenario might see different results, or I might get a slightly different run tomorrow and convince myself the system is better or worse than it is.

Each benchmark is a Python script that subscribes to the live ROS2 topics, collects data under consistent conditions, runs the measurement, and outputs a result card. Same script, same scenario, same hardware — you get the same result. That's the point.

## The Objectives

The full set of objectives for this project is:

1. **Real-Time Visual Perception** — YOLO26s cone detection (blue, yellow, small orange, large orange) at mAP@50 ≥85% and ≥30 FPS.
2. **Perception Localisation Accuracy** — ≥95% cone detection rate with ≤0.20 m positional error at 20–30 m range.
3. **Multi-Modal Sensor Fusion** — camera-LiDAR fusion combining visual classification with depth measurement.
4. **ROS2 Data Synchronisation** — temporal alignment between asynchronous camera and LiDAR sensors.
5. **Simulation Validation** — validate the full pipeline in IPG CarMaker, evaluating accuracy, localisation, and computational efficiency.
6. **SLAM Feasibility Study** — examine SLAM methodologies in Formula Student literature and assess integration pathways.

Objectives 5 and 6 are addressed differently. Objective 5 — simulation validation — is the context everything here runs inside; running the benchmarks in CarMaker with ground truth available for comparison is the validation. Objective 6 — the SLAM feasibility study — was covered in Week 9: the research into EKF-SLAM and graph-SLAM, the mapping architecture that ended up running, and the honest assessment of what was and wasn't built.

Objective 1 splits into two separate measurements since accuracy and speed are independent. That gives five benchmarks across objectives 1 through 4.

## Five Benchmarks

**Objective 1a — mAP@50** comes from the YOLO training logs — the model was already evaluated against the FSOCO validation set during training, so this is more analysis than new measurement. What the script does is load the results from all four trained models and plot them against each other and against the 85% target line.

**Objective 1b — FPS** is a separate benchmark because CarMaker only delivers frames at around 3 Hz, which says nothing about the detector's actual throughput. The script collects 100 real frames from the simulation, loads them into memory, warms up the GPU, then runs 5 independent timed passes over the same frames. The simulation clock is completely out of the picture.

**Objective 2 — Localisation Accuracy** runs the full pipeline in the Acceleration scenario, waits for the car to drive through a straight of cone gates, then takes a simultaneous snapshot of the ground truth positions, the odometry, and the mapped cone positions. Euclidean nearest-neighbour matching pairs each mapped cone to its ground truth counterpart, and the lateral and longitudinal errors are computed gate by gate.

**Objective 3 — Fusion Range** captures a snapshot of all four relevant topics at once — the ground truth, the YOLO detections, the raw LiDAR clusters, and the fused output — and finds the furthest cone each modality successfully resolved. One thing that needed handling: the LiDAR publishes positions relative to `Lidar_F` (the front bumper), while the localizer publishes relative to `Fr1A` (the rear axle). There's a 2.9 m offset between them, so the coordinate frames get normalised before comparing.

**Objective 4 — End-to-End Latency** timestamps every message arrival at every stage — raw camera frame in, raw LiDAR scan in, YOLO detection out, DBSCAN cluster out, fused cone out — and computes the wall-clock latency per stage. Runs 3 trials of 100 frames each with a 5-second cooldown between trials to let the pipeline settle.

## The Test Environment

Everything runs in the CarMaker Acceleration scenario with a straight track laid out using a TCL cone-generation script — 15 gate pairs of blue and yellow cones, evenly spaced, out to around 30 m. The car drives the straight at a consistent speed, sensors stream to the Jetson over the same Ethernet link used in development, and the perception pipeline runs exactly as it would in any other scenario. Nothing is artificially simplified for the test.

The Jetson is the hardware that matters here. Running these benchmarks on a laptop or a desktop would produce numbers that don't reflect where this system actually has to run, so all measurements are taken on the target platform.

## This Week

The results come out one objective at a time. Tomorrow covers Objective 1 — the detector accuracy and speed numbers. Wednesday is the localisation accuracy result, which is the one I was most uncertain about going in. Thursday covers fusion range and end-to-end latency. Friday is the full picture.
