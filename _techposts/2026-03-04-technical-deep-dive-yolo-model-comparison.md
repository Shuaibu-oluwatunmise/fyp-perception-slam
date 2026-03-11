---
title: "Technical Deep Dive: Four Models, One Dataset — A YOLO Architecture Comparison"
date: 2026-03-04 18:01:00 +0000
categories: [Week 7, Phase 4, Technical]
tags: [yolo, yolov8, yolo26, machine-learning, object-detection, training, mAP, NMS]
pin: false
---

Four YOLO models trained on the same FSOCO dataset, same configuration, same 100-epoch budget. This post documents the per-model analysis and the architectural reasoning behind the results.

---

## Training Configuration

All four models were trained with identical settings:

```yaml
Dataset:    FSOCO (Formula Student Objects in Context)
Task:       Multi-class cone detection (blue, yellow, orange, large_orange)
Epochs:     100
Batch size: 32
Image size: 640x640
Optimizer:  AdamW
AMP:        Enabled (mixed precision)
```

The same data augmentation strategy was applied across all runs — HSV jitter, horizontal flip, mosaic, random erasing — to ensure the comparison reflects model architecture rather than training setup differences.

---

## Model 1: YOLOv8n — The Baseline

**Final metrics:** mAP@50 = 78.4% | mAP@50-95 = 54.3% | Precision = 91.8% | Recall = 71.6%

![YOLOv8n training curves](/assets/img/techposts/2026-03-04/results_yolov8n.png)
_YOLOv8n training and validation curves across 100 epochs._

### Training Curve Observations

Box loss drops from 1.66 → 0.86 (48% reduction). Classification loss drops from 1.81 → 0.35 (81% reduction). DFL loss runs in the **0.76–0.85 range** — this scale is characteristic of the NMS-dependent YOLOv8 architecture, where DFL optimises bounding box coordinate distributions across multiple anchor predictions per object.

Validation losses track training losses throughout with no divergence — no overfitting. The model hits a representational ceiling around epoch 70 and flattens.

### Precision vs Recall

91.8% precision: when YOLOv8n fires, it's almost always right. 71.6% recall: it misses roughly 1 in 4 cones. The misses are concentrated in:
- Cones beyond 25–30m (sub-20-pixel apparent size at 640×640)
- Partial occlusions (requires >50% visibility)
- Low-contrast backgrounds

This precision/recall imbalance is the known weakness flagged in the Week 3 analysis.

### Validation Predictions

![YOLOv8n validation predictions](/assets/img/techposts/2026-03-04/val_pred_yolov8n.jpg)
_YOLOv8n validation batch — note the scattered confidence scores (0.3, 0.8, 0.9) across the same frame._

Confidence scores are inconsistent — the same scene produces a 0.3 and a 0.9 detection in adjacent cones. The model is uncertain about objects it should be confident about.

---

## Model 2: YOLO11n — No Meaningful Gain

**Final metrics:** mAP@50 = 78.5% | mAP@50-95 = 54.2% | Precision ≈ 91% | Recall ≈ 71%

![YOLO11n training curves](/assets/img/techposts/2026-03-04/results_yolo11n.png)
_YOLO11n training curves — near-identical shape to YOLOv8n._

The training curves are functionally identical to YOLOv8n. DFL loss in the same 0.76–0.86 range. Same NMS-based architecture. Converges faster (epoch 83 vs 91) but lands at the same ceiling.

The +0.04% mAP@50 over YOLOv8n is statistical noise. YOLO11n offers no practical improvement for this dataset and task.

**Verdict:** Dead end at the nano scale with NMS-based architecture.

---

## Model 3: YOLO26n — Architectural Shift

**Final metrics:** mAP@50 = 83.5% | mAP@50-95 = 58.4% | Precision ≈ 90% | Recall ≈ 74%

![YOLO26n training curves](/assets/img/techposts/2026-03-04/results_yolo26n.png)
_YOLO26n training curves — note the DFL loss scale change._

### What Is DFL Loss?

Distribution Focal Loss (DFL) is the loss function YOLO uses for bounding box edge prediction. Traditional detectors predict a single value for each box coordinate — "the right edge is at pixel 150". DFL instead makes the model predict a **probability distribution** over a range of possible values, then computes the weighted average (expected value) as the final prediction.

The advantage: instead of forcing the model to commit to one pixel with a hard regression, DFL lets it express uncertainty. If a box edge is blurry or ambiguous, the distribution will be spread out. If it's confident, the distribution will spike sharply at one value. The gradients are richer and more stable as a result.

### The Scale Change

The most important thing visible in the YOLO26n curves is the **DFL loss scale**: it runs in the **0.0008–0.0022 range** — three orders of magnitude smaller than YOLOv8n's 0.76. This is not a tuning difference. It is the architectural signature of YOLO26's End-to-End (NMS-free) design.

In standard YOLO models, each grid cell predicts **multiple overlapping candidate boxes**. DFL operates on all of them — the distributions are wide and noisy, and the loss accumulates across every redundant prediction. NMS then suppresses the duplicates post-inference.

YOLO26 trains the network to produce **exactly one box per object**. DFL only needs to refine that single, already-committed prediction — the distribution is narrow and the loss signal is small by design.

### Performance

+5.1 percentage points over YOLOv8n at mAP@50. The model was still improving at epoch 99 (the cutoff) — hadn't fully plateaued. With a larger patience window it may have gained another 0.5–1%.

---

## Model 4: YOLO26s — The Winner

**Final metrics:** mAP@50 = 88.0% | mAP@50-95 = 63.7% | Precision ≈ 93%+ | Recall ≈ 80%+

![YOLO26s training curves](/assets/img/techposts/2026-03-04/results_yolo26s.png)
_YOLO26s training curves — highest precision, highest recall, converges earlier than YOLO26n._

### Why Small Beats Nano

YOLO26s has roughly 5–6M parameters vs YOLO26n's ~3M. The additional capacity allows the model to generalise better from the FSOCO dataset — more representational depth to handle the full range of cone sizes, lighting conditions, and distances present in the training data. Notably, it converges 9 epochs earlier than YOLO26n while landing 4.5 points higher.

The recall jump is the key result: **~80%+ vs 71.6% for YOLOv8n**. This is the long-range detection improvement the whole comparison was designed to test. YOLOv8n was missing approximately 1 in 4 cones. YOLO26s cuts that miss rate roughly in half.

### Validation Predictions

![YOLO26s validation predictions](/assets/img/techposts/2026-03-04/val_pred_yolo26s.jpg)
_YOLO26s validation batch — detections near 1.0 confidence across the board._

The confidence distribution has fundamentally changed. Where YOLOv8n produced scattered 0.3–0.9 scores across the same frame, YOLO26s produces near-1.0 across the board. The model is decisive — a direct result of training without NMS as a safety net.

### Confusion Matrices — Side by Side

| YOLOv8n | YOLO26s |
| :---: | :---: |
| ![YOLOv8n confusion matrix](/assets/img/techposts/2026-03-04/confusion_yolov8n.png) | ![YOLO26s confusion matrix](/assets/img/techposts/2026-03-04/confusion_yolo26s.png) |

Reading the diagonal (correct detections per class) and the background row (missed — classified as background):

| Class | YOLOv8n Correct | YOLOv8n Missed | YOLO26s Correct | YOLO26s Missed | Improvement |
|---|---|---|---|---|---|
| Blue cone | 73% | 27% | 79% | 21% | **+6pp** |
| Yellow cone | 73% | 27% | 80% | 20% | **+7pp** |
| Orange cone | 77% | 22% | 84% | 15% | **+7pp** |
| Large orange cone | 79% | 21% | 81% | 17% | **+4pp** |

Every class improved. The miss rate — how often the model sees a cone and classifies it as background — dropped across the board. Orange and yellow see the biggest gains, which directly corresponds to the long-range detection improvement: these are the standard boundary cones that appear small and distant first.

Cross-class confusion (blue predicted as yellow, or orange as large orange) remains near zero in both models — the colour discrimination was already solid in YOLOv8n and YOLO26s maintains it.

---

## The NMS-Free Architecture Explained

Standard YOLO inference pipeline:
```
GPU: forward pass → hundreds of overlapping candidate boxes
CPU: NMS post-processing → suppress duplicates
Output: cleaned detections
```

YOLO26 inference pipeline:
```
GPU: forward pass → one box per object (trained end-to-end)
Output: detections (no CPU step)
```

Practical implications for this system:

**1. No CPU latency spike.** On edge hardware like the Jetson AGX Orin, standard NMS runs on the CPU after the GPU finishes. This adds 5–10ms per frame. YOLO26 eliminates this. For a system targeting real-time fusion, this is meaningful headroom.

**2. More decisive predictions.** Because the model is trained to be right with one box (no NMS fallback), it develops higher inherent confidence. The validation prediction comparison above shows this directly.

**3. No class-aware suppression issues.** Standard NMS suppresses overlapping boxes of the *same class*. Two cones of different colours in close proximity are handled independently — but two cones of the same colour within the NMS IoU threshold can suppress each other. YOLO26's end-to-end design avoids this entirely. This is directly related to the Ghost Box issue in the fusion engine (covered in Thursday's post).

---

## Summary Comparison

| Metric | YOLOv8n | YOLO11n | YOLO26n | YOLO26s |
|---|---|---|---|---|
| mAP@50 | 78.4% | 78.5% | 83.5% | **88.0%** |
| mAP@50-95 | 54.3% | 54.2% | 58.4% | **63.7%** |
| Precision | ~91.8% | ~91% | ~90% | **~93%+** |
| Recall | ~71.6% | ~71% | ~74% | **~80%+** |
| Converges | Epoch 91 | Epoch 83 | Epoch 99 | **Epoch 90** |
| NMS-free | No | No | Yes | **Yes** |
| DFL scale | ~0.76 | ~0.76 | ~0.001 | **~0.001** |

## Decision

YOLO26s replaces YOLOv8n as the production camera detector. It is deployed in the fusion pipeline from this point forward.
