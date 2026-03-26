---
title: "Does It Meet the Objectives?"
date: 2026-03-27 00:00:00 +0000
categories: [Week 10, Testing & Evaluation]
tags: [evaluation, benchmarking, perception, yolo26s, fusion, localisation, latency]
pin: false
image:
  path: /assets/img/thumbnails/43.png
  alt: Evaluation Summary
  header: false
---

Five benchmarks across four objectives. This is where they all land together.

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

Five pass, one fails — though one of those passes has a caveat. The numbers tell different stories depending on which one you look at.

## What Went Better Than Expected

The lateral localisation accuracy is the headline result. 1.7 cm mean against a 20 cm target is not a narrow pass — it's a 12x margin. The fusion architecture — projecting LiDAR into image space and matching against YOLO boxes using the Hungarian algorithm — is doing exactly what it was designed to do. The position of a cone in the vehicle frame is essentially where it should be.

Detection rate hitting 100% is also worth noting. The objective asked for 95% and every cone in the scenario was detected and mapped. In the context of autonomous racing, a missed cone is a track boundary violation — so getting this right matters.

## What Was Tight

The p95 latency of 99.0 ms technically passes — p95 means 95% of frames complete within that time, so the metric is met. But the remaining 5% of frames are taking longer than 99 ms, and some of those will exceed 100 ms. The mean of 80 ms has comfortable headroom, but the tail is close to the edge. Adding TensorRT to the YOLO inference step could bring the p95 down significantly and make this a cleaner result.

The 87.8% mAP@50 passes the ≥85% target, but it's worth being honest about what that number represents. It comes from the FSOCO validation set, which contains challenging conditions the simulation doesn't replicate — varying lighting, partial occlusion, long-range cones. In the simulation environment the detector performed better than the validation metrics suggest. The 87.8% is the harder, more honest number.

## What Didn't Meet the Target

The longitudinal positional error is the honest failure in this evaluation. Up to ~1.5 m under motion against a 0.20 m target is not close. A follow-up investigation confirmed the relationship is linear with speed — a fit across two runs at different speed profiles gave an implied effective latency of ~130 ms and an R² of 0.658. At 10 m/s the forward bias is around 1.3 m. The most consistent explanation is the local mapper's confirmation mechanism adding delay on top of the raw sensor latency, though other factors weren't isolated and the cause hasn't been fully pinned down. On a straight track the offset matters less, but on a turn it directly affects where the path planner thinks the corner apex is. That's not something that can be reasoned away.

## What This Means

The perception system does what it was designed to do within the simulation environment. It detects cones reliably, assigns the right colours, places them accurately in 3D space, and does all of it fast enough to be useful in real time — on the actual target hardware, not a development machine. The LiDAR detection window runs from 0.2 m to 25.1 m — the near-field limit doesn't affect the Acceleration scenario but is a real constraint for tighter track layouts like the Skidpad.

That's Objective 5 too — simulation validation. The pipeline has been validated in IPG CarMaker under a controlled scenario with ground truth available for comparison, and the results are measurable and reproducible.

What it doesn't tell you is how the system would behave on a real vehicle with real sensor noise, calibration drift, and weather. Simulation is controlled by design. The numbers here are a foundation, not a guarantee.
