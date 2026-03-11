---
title: "The Model Showdown"
date: 2026-03-04 18:00:00 +0000
categories: [Week 7, Phase 4]
tags: [yolo, yolov8, yolo26, machine-learning, object-detection, perception]
pin: false
image:
  path: /assets/img/thumbnails/30.png
  alt: YOLO26s Confusion Matrix
  header: false
---
I woke up early today and decided to train three more models.

The YOLOv8n baseline has been running since Phase 1. It works — but back in the [Week 3 training post](/fyp-perception-slam/posts/training-yolo-cone-detector/), I flagged that its biggest weakness was long-range detection. Cones beyond 25–30 meters appear as sub-20-pixel objects at 640×640, and YOLOv8n misses a significant fraction of them. The plan was always to come back and test YOLO26 to see if its architecture improvements actually addressed this. Today was that day.

I trained three new models on the same FSOCO dataset, same training configuration:

- **YOLO11n** — the latest nano from Ultralytics
- **YOLO26n** — YOLO26 nano variant
- **YOLO26s** — YOLO26 small variant

Then ran a comparison against the original YOLOv8n.

## The Results

| Model             | mAP@50          | mAP@50-95       |
| ----------------- | --------------- | --------------- |
| YOLOv8n           | 78.4%           | 54.3%           |
| YOLO11n           | 78.5%           | 54.2%           |
| YOLO26n           | 83.5%           | 58.4%           |
| **YOLO26s** | **88.0%** | **63.7%** |

YOLO11n was a dead end — essentially identical to YOLOv8n. The step up to YOLO26 architecture is where things changed. The nano variant gains about 5 percentage points. The small variant gains nearly 10.

The number that matters most for this project is recall — the percentage of actual cones the model detects. From the training curves, YOLOv8n's recall sits at approximately 71.6%. YOLO26s comes in at ~80%+. That improvement is directly addressing the long-range detection weakness flagged in the [Week 3 training analysis](/fyp-perception-slam/techposts/technical-deep-dive-yolo-model-comparison/).

## Seeing It in Action

The raw numbers are one thing. Running both models on the simulation and comparing side by side is another.

**Acceleration event — YOLOv8n:**

![YOLOv8n on the Acceleration event](/assets/img/blog/2026-03-04/ac8.png)
_Detection drops off quickly down the straight. Near cones fine, distant cones sparse._

**Acceleration event — YOLO26s:**

![YOLO26s on the Acceleration event](/assets/img/blog/2026-03-04/ac26.png)
_Detection extends further down the straight. YOLO26s also correctly identifies the large orange cones as `large_orange` — YOLOv8n was misclassifying them._

**TrackDrive event — YOLOv8n:**

![YOLOv8n on the TrackDrive event](/assets/img/blog/2026-03-04/td8.png)
_Mid-range detections get cluttered. Overlapping lower-confidence boxes at distance._

**TrackDrive event — YOLO26s:**

![YOLO26s on the TrackDrive event](/assets/img/blog/2026-03-04/td26.png)
_Cleaner detections, consistent confidence scores, cones visible further down the track._

## The Confusion Matrices

|                                   YOLOv8n                                   |                                   YOLO26s                                   |
| :--------------------------------------------------------------------------: | :--------------------------------------------------------------------------: |
| ![YOLOv8n confusion matrix](/assets/img/blog/2026-03-04/confusion_yolov8n.png) | ![YOLO26s confusion matrix](/assets/img/blog/2026-03-04/confusion_yolo26s.png) |

The diagonal tells the story. In YOLOv8n, blue and yellow cones are correctly detected 73% of the time each — the remaining 27% are classified as background (missed). In YOLO26s, blue rises to 79%, yellow to 80%, orange to 84%, large orange to 81%. Every class improved, and crucially, the background row — the cones the model is failing to detect — shrinks across all four classes.

This is what improved recall looks like in practice.

## Why YOLO26 Performs Differently

YOLO26 is End-to-End — it is trained to output exactly one box per object, without any post-processing suppression step. Standard models like YOLOv8 generate hundreds of overlapping candidate boxes and rely on Non-Maximum Suppression (NMS) running on the CPU to clean them up. YOLO26 eliminates that step entirely, which means no CPU bottleneck and more decisive predictions.

The confidence score distribution tells this story clearly. In the YOLOv8n validation predictions, you see 0.3, 0.8, and 0.9 on different cones in the same frame — it's inconsistent. In YOLO26s, most detections come in near 1.0. The model knows what it's looking at.

The full architectural breakdown, training curves for all four models, and per-class analysis is in the **[Technical Deep Dive](/fyp-perception-slam/techposts/technical-deep-dive-yolo-model-comparison/)**.

## Decision

YOLO26s replaces YOLOv8n as the production camera detector going forward. The mAP gain, the recall improvement, and the NMS-free architecture all point in the same direction. This is the model going into the fusion engine.

Tomorrow, I'll be running the new model through more stress tests — different scenarios, edge cases, see what else surfaces before touching the fusion engine.
