---
title: "Building Node 2a: Implementing Real-Time Cone Detection"
date: 2026-02-04 17:00:00 +0000
categories: [Week 3]
tags: [ros2, yolo, perception, opencv, debugging]
pin: false
---

The YOLOv8 model was trained. Now it needed to run in the ROS 2 pipeline.

The goal: take the video feed from Node 1a, run it through YOLO, and output bounding boxes. **Node 1a (Camera) → Node 2a (Detector) → Bounding Boxes.**

## Implementation

I kept Node 2a **stateless**. It doesn't track cones across frames or remember what it saw before. Each frame is processed independently—look at the image, find the cones, publish the detections, forget everything, repeat.

Why? Because I wanted it simple and robust. If it crashes, it restarts instantly. No "state" to recover. No complex initialization. Just pure detection.

The implementation uses three key pieces:
- **Ultralytics** for YOLOv8 inference
- **cv_bridge** to translate between ROS images and OpenCV
- **vision_msgs** for standard detection messages

Here's the node working correctly:

{% include embed/youtube.html id='A5wHdat6QBc' %}
_Node 2a detecting and classifying cones at 30 FPS_

## The Color Channel Bug

When I first ran the node, something felt off immediately. The detections were coming through with solid confidence scores—85%, 90%—but the **classifications were completely wrong**. Yellow cones were being labeled as blue. Blue cones as yellow. The AI was confidently wrong.

I pulled up the visualization output and saw the problem instantly—the colors were inverted:

{% include embed/youtube.html id='DvQaQJ1TnZs' %}
_The color bug: Yellow cones appear blue to the AI, blue cones appear orange. High confidence, wrong class._

It hit me: **BGR vs RGB encoding**.

### The Root Cause

This is one of those bugs that makes you feel stupid and enlightened at the same time.

OpenCV and YOLO expect images in **BGR (Blue-Green-Red)** format. ROS 2 image messages use **RGB (Red-Green-Red)** format. When I accidentally used the wrong encoding in `cv_bridge`, the model saw inverted colors:

- **Yellow cones** → Appeared **BLUE** to the AI
- **Blue cones** → Appeared **ORANGE** to the AI

The critical line:

```python
cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
```

This tells `cv_bridge` to convert the incoming RGB message to BGR format for YOLO. Without this conversion, the model sees the wrong colors entirely.

### The Fix

Once I corrected the encoding, the classifications snapped into place. Yellow cones were correctly labeled as yellow, blue as blue. The model could finally see the world correctly.

This bug taught me something important: **always visualize your data pipeline**. If I hadn't looked at the debug image stream, I might have spent days retraining the model or tweaking hyperparameters, when the issue was a single line of code.

## Performance

The node runs at ~30ms per frame on the Jetson AGX Orin (with CUDA). That's 30+ FPS, which is more than enough for the 30 Hz camera feed.

The bottleneck isn't the inference itself—it's the image serialization and deserialization between ROS and OpenCV. The actual YOLO forward pass is fast.

The node publishes two topics:
- `/detections` — Standard detection messages for downstream nodes
- `/detections_image` — Visualization with bounding boxes (useful for debugging)

> **For comprehensive technical details on Node 2a's architecture, BGR/RGB handling, deep learning concepts, and troubleshooting, see the [Technical Deep-Dive: Node 2a Camera Cone Detector](/fyp-perception-slam/techposts/node-2a-camera-detector/).**

## Next Steps

Node 2a now reliably detects and classifies cones in 2D image space. But the path planner doesn't care about pixels—it needs real-world positions.

The next challenge is **Node 3**: converting these 2D pixel coordinates `(u, v)` into 3D positions `(x, y, z)` in the vehicle's coordinate frame.

That requires camera intrinsics, depth data, and coordinate transformations. I'll tackle that tomorrow.
