---
title: "Training YOLO for Cone Detection: Results and Next Steps"
date: 2026-02-03 10:00:00 +0000
categories: [Technical Deep-Dive, Week 3]
tags: [yolo, cone-detection, machine-learning, training, fsoco]
pin: false
---

Yesterday, I planned the approach for Node 2a. Today, I trained the model.

## Results

It works.

{% include embed/youtube.html id='eHl35BZ97u0' %}
_Demo: YOLOv8n detecting cones on Formula Student footage_

The model achieves **78.3% mAP@0.5** with **91.8% precision**. That means when it says "there's a cone," it's right 9 out of 10 times. For autonomous racing, that's critical—false positives (phantom cones) could cause dangerous steering corrections.

The recall is **71.6%**, which means it detects about 7 out of 10 cones. The ones it misses are primarily distant cones (beyond 25-30 meters) or partially occluded ones. That's acceptable for Phase 1, especially since the SLAM system will help fill in gaps across multiple frames, and LiDAR fusion will catch what the camera misses.

## The Training Process

I used the **FSOCO (Formula Student Objects in Context)** dataset—thousands of labeled images from real Formula Student competitions with varying weather, lighting, and track conditions.

Training configuration:
- **Model:** YOLOv8n (nano variant, 3.2M parameters)
- **Epochs:** 100
- **Batch size:** 32
- **Dataset:** FSOCO (blue, yellow, orange, large orange cones)

![Training Curves](/assets/img/blog/2026-02-03/results.png)
_Training and validation metrics over 100 epochs_

The training converged smoothly with no overfitting. Validation loss tracked training loss closely, which means the model generalizes well to unseen data. The augmentation strategy (color jittering, flips, mosaic) worked as intended.

## What I Learned

**1. High precision is more important than high recall for autonomous racing**

91.8% precision means minimal false positives. I'd rather miss a few distant cones than have the car swerve to avoid phantom obstacles. The SLAM system will build a map over time, so missed cones in one frame can be caught in the next.

**2. The model has excellent color discrimination**

Less than 1% confusion between blue and yellow cones. This is critical—blue marks the left track boundary, yellow marks the right. Confusing them would send the car off-track. The model reliably distinguishes these, even in challenging lighting.

**3. Distance is the main limitation**

Most missed detections are cones beyond 25-30 meters. At that distance, cones appear as small objects (less than 20 pixels), which is below the reliable detection threshold. This is a known limitation of the 640x640 input resolution. When I test YOLO26 later, I'll see if its architecture improvements help with long-range detection.

> **For a comprehensive analysis of training metrics, confusion matrices, and per-class performance breakdowns, see the [Technical Deep-Dive: YOLOv8n Training Analysis](/fyp-perception-slam/techposts/yolov8n-training-analysis/).**

## Training Resources

For those interested in the technical details or wanting to reproduce/extend this work:

 **[Download Trained Model (best.pt)](/fyp-perception-slam/assets/docs/blog/2026-02-03/best.pt)** (6.3 MB)  
_YOLOv8n weights trained on FSOCO dataset for Formula Student cone detection_

 **[Training Configuration (args.yaml)](/fyp-perception-slam/assets/docs/blog/2026-02-03/args.yaml)**  
_Complete hyperparameters and augmentation settings_

 **[Training Metrics (results.csv)](/fyp-perception-slam/assets/docs/blog/2026-02-03/results.csv)**  
_Per-epoch loss, mAP, precision, and recall data_

## Next Steps

The model is trained and validated. Tomorrow, I integrate it into **Node 2a** (the ROS 2 detection node) and test the full pipeline end-to-end.

This is where things get real. Up until now, I've been working with isolated components—a camera publisher here, a trained model there. Tomorrow, I connect them and see if the perception pipeline actually works as a system.

If it does, I'll have a working camera-based cone detector publishing detections to ROS 2 topics at 30 Hz. That's Phase 1 really close to completion.
