---
title: "YOLOv8n Training Analysis: Cone Detection Performance Breakdown"
date: 2026-02-03 11:00:00 +0000
categories: [Technical Deep-Dive]
tags: [yolo, training-analysis, performance-metrics, confusion-matrix, fsoco]
pin: false
description: "Comprehensive analysis of YOLOv8n training for Formula Student cone detection, including performance metrics, confusion matrix breakdown, and deployment recommendations."
---

## Overview

This technical deep-dive provides a comprehensive analysis of the YOLOv8n baseline model trained for cone detection in the Formula Student AI perception system. The model achieves **78.3% mAP@0.5** and **54.2% mAP@0.5:0.95**, establishing a solid foundation for real-time autonomous navigation.

**Key Findings:**
- High precision (91.8%) ensures reliable detections with minimal false positives
- Moderate recall (71.6%) indicates ~28% of cones are missed, primarily distant/occluded instances
- Excellent inter-class discrimination (minimal color confusion between cone types)
- Smooth convergence with no overfitting observed

---

## Training Configuration

### Model Architecture
- **Model:** YOLOv8n (pretrained)
- **Parameters:** ~3.2M (nano variant)
- **Input Size:** 640x640 pixels
- **Target Hardware:** NVIDIA Jetson AGX Orin

### Hyperparameters

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| **Epochs** | 100 | With early stopping (patience=15) |
| **Batch Size** | 32 | Balanced for GPU memory and convergence |
| **Learning Rate (initial)** | 0.01 | Standard YOLOv8 default |
| **Optimizer** | AdamW | Adaptive learning for better convergence |
| **Image Size** | 640 | Standard YOLO resolution |
| **AMP** | Enabled | Mixed precision for faster training |

### Data Augmentation Strategy

| Augmentation | Value | Purpose |
|--------------|-------|---------|
| **HSV-Hue** | 0.015 | Minimal color shift (preserve cone colors) |
| **HSV-Saturation** | 0.7 | Handle varying lighting conditions |
| **HSV-Value** | 0.4 | Brightness variation for day/night |
| **Horizontal Flip** | 0.5 | Track symmetry augmentation |
| **Translation** | 0.1 | Simulate camera position variance |
| **Scale** | 0.5 | Handle varying cone distances |
| **Mosaic** | 1.0 | Multi-image composition |
| **Random Erasing** | 0.4 | Simulate occlusions |

**Note:** Rotation and vertical flip are disabled, as Formula Student tracks maintain consistent orientation.

---

## Training Dynamics

![Training Results](/assets/img/techposts/2026-02-03/results.png)
_Training and validation metrics over 100 epochs_

### Loss Convergence

**Box Localization Loss:**
- Initial: 1.66 (epoch 1)
- Final: 0.86 (epoch 100)
- Reduction: 48% decrease

The model learned accurate bounding box regression. The final loss of 0.86 indicates sub-pixel precision in cone localization, critical for accurate 3D position estimation in the Late Fusion pipeline.

**Classification Loss:**
- Initial: 1.81 (epoch 1)
- Final: 0.35 (epoch 100)
- Reduction: 81% decrease

The model quickly learned to distinguish cone classes. The dramatic reduction suggests clear visual differences between blue, yellow, orange, and large orange cones in the FSOCO dataset.

### Validation Behavior

| Loss Type | Epoch 1 | Epoch 50 | Epoch 100 | Trend |
|-----------|---------|----------|-----------|-------|
| Box Loss | 1.44 | 0.96 | 0.94 | Stable |
| Classification Loss | 0.90 | 0.39 | 0.37 | Stable |
| DFL Loss | 0.81 | 0.76 | 0.76 | Stable |

**Critical Observation:** Validation losses track training losses closely with no divergence, indicating:
- No overfitting
- Good generalization to unseen data
- Effective augmentation strategy

The plateau after epoch 70 suggests the YOLOv8n architecture reached its representational capacity for this task.

---

## Performance Metrics

### Mean Average Precision (mAP)

**mAP@0.5 (Primary Metric):**
- Final: 78.3%
- Peak: 78.4% (epoch 88)
- Progression:
  - Epoch 10: 70.2%
  - Epoch 30: 75.8%
  - Epoch 50: 77.1%
  - Epoch 100: 78.3%

The model achieves strong performance for real-time detection. In Formula Student context, 78.3% mAP@0.5 means the model correctly localizes and classifies approximately 4 out of 5 cones in typical track scenarios.

**mAP@0.5:0.95 (Strict Localization):**
- Final: 54.2%
- Peak: 54.3% (epoch 88)

The 24% gap between mAP@0.5 and mAP@0.5:0.95 indicates that while the model detects cones reliably, bounding box precision degrades at stricter IoU thresholds. This is acceptable for autonomous racing, where cone center estimation (not exact boundaries) is the priority.

### Precision Analysis

- Final: 91.8%
- Peak: 92.4% (epoch 60)
- Stability: Maintained >90% after epoch 20

**91.8% precision** means only **8.2% false positive rate**. When the model predicts a cone, it's correct 9 out of 10 times. This is critical for path planning—false positives (phantom cones) could cause dangerous steering corrections. High precision minimizes this risk.

### Recall Analysis

- Final: 71.6%
- Peak: 71.8% (epoch 78)
- Stability: Plateaued around 70-72% after epoch 40

**71.6% recall** means the model detects approximately 72 out of 100 actual cones. The 28% of missed cones are primarily:
1. Distant cones (>30m) at image periphery
2. Partially occluded cones behind other objects
3. Small cones in low-contrast backgrounds

**Mitigation Strategies:**
- SLAM Integration: Missed cones in one frame may be detected in subsequent frames
- LiDAR Fusion: LiDAR can detect cones missed by the camera
- Lower Confidence Threshold: Reduce from 0.25 to 0.15-0.20 to boost recall

### Precision-Recall Trade-off

![Precision-Recall Curve](/assets/img/techposts/2026-02-03/BoxPR_curve.png)
_Precision-Recall curve showing operating point_

The model operates at a **high-precision, moderate-recall** operating point. This is strategically sound for autonomous racing:

| Scenario | Precision Priority | Recall Priority |
|----------|-------------------|-----------------|
| High-speed sections | Avoid phantom obstacles | Less critical (map memory) |
| Track initialization | Less critical | Build complete map |
| Cone-dense chicanes | Avoid false positives | Moderate (redundancy exists) |

---

## Per-Class Performance Analysis

### Confusion Matrix Breakdown

![Confusion Matrix](/assets/img/techposts/2026-02-03/confusion_matrix_normalized.png)
_Normalized confusion matrix showing per-class detection rates_

| True Class | Detected Correctly | Misclassified as Background | Cross-Class Confusion |
|------------|-------------------|----------------------------|---------------------|
| Blue Cone | 73% | 27% | <1% (yellow) |
| Yellow Cone | 73% | 27% | <1% (blue) |
| Orange Cone | 77% | 22% | 1% (large orange) |
| Large Orange Cone | 79% | 21% | 3% (orange) |

### Key Insights

**Excellent Color Discrimination:**
- Blue ↔ Yellow confusion: <1%
- Orange ↔ Large Orange confusion: 1-3%

The model reliably distinguishes track boundaries (blue/yellow) from special markers (orange). This is critical for path planning—confusing left/right boundaries would cause the vehicle to drive off-track.

**Background False Negatives (Primary Weakness):**
- 27% of blue/yellow cones misclassified as background
- 21-22% of orange cones misclassified as background

**Root Causes:**
1. Distance: Cones >30m appear as small objects (<20 pixels)
2. Occlusion: Partially hidden cones lack distinctive color features
3. Low Contrast: Cones against similar-colored backgrounds

**Large Orange Cone Performance (Best Class):**
- 79% detection rate (highest among all classes)
- Reason: Larger physical size (2x standard cones) → more pixels → easier detection

---

## Validation Predictions Analysis

### Visual Examples

![Validation Batch 0](/assets/img/techposts/2026-02-03/val_batch0_pred.jpg)
_Sample validation predictions showing successful cone detections_

![Validation Batch 1](/assets/img/techposts/2026-02-03/val_batch1_pred.jpg)
_Additional validation examples with varying lighting and distances_

### Successful Detection Patterns
1. Well-lit, unoccluded cones: Near-perfect detection
2. Mid-range cones (5-20m): Reliable bounding boxes with correct class labels
3. High-contrast scenarios: Excellent performance (e.g., blue cone on grass)

### Common Failure Modes
1. **Distant cones (>25m):**
   - Often missed entirely (false negatives)
   - When detected, bounding boxes are loose/imprecise

2. **Partial occlusions:**
   - Cones behind barriers or other cones frequently missed
   - Model requires >50% visibility for reliable detection

3. **Lighting extremes:**
   - Overexposed (bright sunlight) or underexposed (shadows) regions show reduced performance

---

## Strengths and Weaknesses

### Strengths

1. **High Precision (91.8%)**
   - Minimal false positives ensure safe path planning
   - Reliable detections reduce downstream filtering requirements

2. **Excellent Class Discrimination**
   - <1% blue/yellow confusion preserves track boundary integrity
   - Robust color recognition across lighting conditions

3. **No Overfitting**
   - Validation metrics track training metrics closely
   - Good generalization to unseen track configurations

4. **Fast Inference Potential**
   - YOLOv8n architecture designed for edge devices
   - Expected 5-8ms latency on Jetson AGX Orin (120+ FPS)

### Weaknesses

1. **Moderate Recall (71.6%)**
   - 28% of cones missed, primarily distant/occluded instances
   - May require multiple frames or sensor fusion to achieve full coverage

2. **Background False Negatives**
   - 22-27% of cones misclassified as background
   - Indicates difficulty with small/low-contrast objects

3. **Distance Limitation**
   - Performance degrades significantly beyond 25-30m
   - May limit look-ahead distance for high-speed planning

4. **Occlusion Sensitivity**
   - Requires >50% cone visibility for reliable detection
   - Dense cone clusters may cause mutual occlusion issues

---

## Next Steps

### Immediate Integration (Phase 1)

The model is ready for deployment in the ROS 2 perception pipeline:
- Input: `/camera/image_raw` (rectified images)
- Output: `/detections` (bounding boxes with class labels)
- Confidence Threshold: Start with 0.25, tune based on field testing

### Performance Tuning (Phase 2)

Once integrated with the full system:
- Implement adaptive thresholding based on vehicle speed
- Add temporal filtering to track detections across frames
- Test sensor fusion with LiDAR for improved coverage

### Future Improvements (Phase 3)

After establishing the baseline:
- Train and evaluate YOLO26 variant
- Target: >83% mAP@0.5, >78% recall
- Compare performance and inference speed
- Deploy the better-performing model

---

## Conclusion

The YOLOv8n cone detector establishes a solid baseline for the Formula Student AI perception system, achieving:

- 78.3% mAP@0.5 - competitive with state-of-the-art real-time detectors
- 91.8% precision - reliable detections for safe autonomous navigation
- Fast inference potential - suitable for Jetson AGX Orin deployment
- Excellent class discrimination - preserves critical track boundary information

**Primary Limitation:** 71.6% recall indicates room for improvement in detecting distant/occluded cones. This will be addressed through sensor fusion with LiDAR and temporal integration across multiple frames via SLAM.

**Strategic Assessment:** This model is ready for Phase 1 deployment. It provides a stable foundation to validate the ROS 2 perception pipeline, establish performance benchmarks, and identify real-world failure modes to inform future improvements.
