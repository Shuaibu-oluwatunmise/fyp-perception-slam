---
title: "Objective 1: Does the Detector Hold Up?"
date: 2026-03-24 17:00:00 +0000
categories: [Week 10, Testing & Evaluation]
tags: [yolo, yolo26s, map, fps, benchmarking, evaluation, object-detection]
pin: false
image:
  path: /assets/img/thumbnails/40.jpg
  alt: YOLO26s Detector Results
  header: false
---

Objective 1 has two parts: accuracy and speed. They sound related but they're measured completely independently — a model can be highly accurate and too slow to be useful, or fast but unreliable. So there are two benchmarks, and they get two results.

## Part 1 — mAP@50

### How It Was Measured

mAP@50 (Mean Average Precision at 50% IoU) is the standard metric for object detection. IoU measures how much a predicted bounding box overlaps with the ground truth box — a threshold of 50% means a detection only counts as correct if the predicted box covers at least half the real cone. The mean is taken across all four cone classes: blue, yellow, small orange, and large orange.

This result doesn't come from a new test run. During training, YOLO evaluates the model against the FSOCO validation set at every epoch and logs the metrics. The benchmark script reads those logs for all four trained models — YOLOv8n, YOLO11n, YOLO26n, and YOLO26s — and plots the training curves and final scores against the ≥85% target.

### The Results

![mAP@50 training curves for all four models](/assets/img/blog/2026-03-24/map50_comparison.png)
_mAP@50 over 100 training epochs. YOLO26s is the only model to clear the 85% threshold._

Three of the four models don't make it. YOLOv8n and YOLO11n both plateau around 78%, and YOLO26n reaches 83.5% — close, but 1.5% short. YOLO26s lands at **87.8%**, which is a pass with 2.8% to spare.

The gap between YOLO26n and YOLO26s is worth noting. They're the same architecture — the only difference is model size (n = nano, s = small). That 4.3% jump from 83.5% to 87.8% is purely from the extra capacity to learn finer features.

![mAP summary card — final scores for all four models](/assets/img/blog/2026-03-24/map_summary_card.png)
_Final epoch scores for all four models. Only YOLO26s passes._

Looking at precision and recall separately:

![Precision curves](/assets/img/blog/2026-03-24/precision_comparison.png)
_Precision over training. All four models converge above 90% — precision isn't the differentiator._

![Recall curves](/assets/img/blog/2026-03-24/recall_comparison.png)
_Recall over training. YOLO26s reaches 80.3% — the others fall short._

Precision is high across all models — they're all good at avoiding false positives. Recall is where the gap opens up. YOLO26s at 80.3% recall means it misses roughly 1 in 5 cones in the validation set. The confusion matrix breaks down where those misses go:

![Normalized confusion matrix for YOLO26s](/assets/img/blog/2026-03-24/confusion_matrix_normalized.png)
_Per-class performance. The background column shows what fraction of each class gets missed entirely._

Cross-class confusion between blue and yellow is essentially non-existent — the model has no trouble telling them apart. There's a very small amount among the orange cone classes, but nothing meaningful. The background column — cones that are present but not detected at all — is where the validation set losses come from, likely from difficult scenarios in the FSOCO dataset like partially occluded cones or very small distant ones. In practice across all the simulation runs over the past few weeks, missed detections were never really an issue. The model sees what it needs to see.

The precision-recall and F1 curves for YOLO26s directly:

![Precision-Recall curve for YOLO26s](/assets/img/blog/2026-03-24/BoxPR_curve.png)
_Precision-Recall curve for YOLO26s across all four cone classes._

![F1-Confidence curve for YOLO26s](/assets/img/blog/2026-03-24/BoxF1_curve.png)
_F1 peaks at 0.86 across all classes at a confidence threshold of 0.251._

The F1 curve shows the model performs best at a confidence threshold of around 0.25. In practice the deployed system uses 0.35 for a bit more conservatism — trading a small amount of recall for cleaner detections.

Finally, a sanity check on what the model actually sees versus what it predicts on the validation set:

![Validation batch ground truth labels](/assets/img/blog/2026-03-24/val_batch0_labels.jpg)
_Ground truth labels on a validation batch._

![Validation batch predictions](/assets/img/blog/2026-03-24/val_batch0_pred.jpg)
_YOLO26s predictions on the same batch. Boxes and colours are consistent with ground truth._

**Objective 1a: PASS — 87.8% mAP@50 against a ≥85% target.**

---

## Part 2 — FPS

### How It Was Measured

CarMaker delivers camera frames at roughly 3 Hz in the current setup — that's how the sensor cluster is configured, not a reflection of what the detector can actually do. Running YOLO at 3 FPS would be meaningless as a benchmark. The solution is to decouple the measurement from the simulation entirely.

The FPS script collects 100 real frames from the live camera topic and loads them into memory. Then it warms up the GPU with 10 inference passes (not timed — GPU cold-start latency is not representative). Then it runs 5 independent timed passes over the same 100 frames using `time.perf_counter()`, computing per-frame latency and overall FPS for each run. Same frames, same model, same hardware — five times.

### The Results

![FPS per run — bar chart](/assets/img/blog/2026-03-24/fps_bar_runs.png)
_FPS across 5 independent runs. All five clear the 30 FPS target._

![Per-frame latency trace across all 5 runs](/assets/img/blog/2026-03-24/fps_line_per_frame.png)
_Per-frame latency for all 500 frames. The 33.3 ms threshold (30 FPS) line is shown. Most frames come in well under it._

![FPS summary card](/assets/img/blog/2026-03-24/fps_summary_card.png)
_Summary card. 35.1 FPS average, ±0.6 variance across runs._

The average across all 5 runs is **35.1 FPS**, with individual runs ranging from 34.5 to 36.0. The ±0.6 FPS variance is about as stable as you'd expect — no thermal throttling, no run-to-run drift, just consistent inference on the Jetson GPU.

The per-frame trace shows occasional spikes above 33.3 ms, which is expected — garbage collection, memory movement, occasional scheduling jitter. None of those individual spikes are a problem as long as the rolling average stays above 30 FPS, and it does throughout every run.

This is also worth stating clearly: YOLO26s is running on raw CUDA with no TensorRT optimisation. TensorRT typically reduces inference latency by 30–50% on Jetson hardware. The 35.1 FPS result is the unoptimised baseline.

**Objective 1b: PASS — 35.1 FPS against a ≥30 FPS target.**

---

Both parts of Objective 1 pass. Tomorrow is Objective 2 — localisation accuracy — which is where things get more interesting.

---

## Resources

**[mAP Analysis Script](/fyp-perception-slam/assets/docs/blog/2026-03-24/obj1_map_analysis.py)** — `obj1_map_analysis.py`
_Loads training logs for all four models, plots curves and summary card_

**[FPS Benchmark Script](/fyp-perception-slam/assets/docs/blog/2026-03-24/obj1_fps_bench.py)** — `obj1_fps_bench.py`
_Collects frames from ROS2, warms up GPU, runs 5 timed inference passes_

**[YOLO26s Training Metrics](/fyp-perception-slam/assets/docs/blog/2026-03-24/yolo26s_results.csv)** — `yolo26s_results.csv`
_Per-epoch loss, mAP@50, precision, and recall over 100 training epochs_
