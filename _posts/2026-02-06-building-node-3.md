---
title: "Building Node 3: Phase 1 Complete!"
date: 2026-02-06 17:00:00 +0000
categories: [Week 3]
tags: [ros2, 3d-localization, perception, phase-1, node-3]
pin: false
image:
  path: /assets/img/thumbnails/11.png
  alt: Building Node 3
  header: false
---

Node 3 is done. The perception pipeline is complete.

I can now take a camera feed, detect cones in real-time, and output their **3D real-world coordinates relative to the camera frame**. This is the foundation everything else builds on.

## The Implementation

The core challenge wasn't the math—the pinhole camera model is straightforward. The challenge was **time synchronization**.

The camera publishes images at 30 FPS. The detector (Node 2a) takes ~30ms to process each frame. If I just grabbed the "latest" depth map whenever a detection arrived, I'd be matching objects from Frame N with depth data from Frame N+1. At racing speeds, that's a significant positioning error.

The solution was `message_filters.ApproximateTimeSynchronizer`. It queues incoming messages and only fires the callback when it finds a detection and depth map with matching timestamps (within 100ms tolerance). This ensures I'm always fusing data from the same instant in time.

## Seeing It Work

Here's the full Phase 1 pipeline running:

{% include embed/youtube.html id='zmT_eKWbDuo' %}
_Phase 1 Complete: Camera → Detection → 3D Localization_

The terminal shows the `/cones_camera_frame` topic publishing 3D positions:

```yaml
cones:
  - position: {x: 2.5, y: 0.3, z: 8.1}
    class_name: "blue_cone"
    confidence: 0.87
    source: "camera"
  - position: {x: -1.2, y: 0.4, z: 12.5}
    class_name: "yellow_cone"
    confidence: 0.92
    source: "camera"
```

The coordinates are in the camera frame:
- `x` = right/left (meters)
- `y` = down/up (meters)  
- `z` = forward distance (meters)

## What I Learned

**1. Time synchronization is non-negotiable**

In multi-sensor systems, you can't just subscribe to topics and process messages as they arrive. You need to match data by timestamp, not arrival time. The `ApproximateTimeSynchronizer` solved this elegantly.

**2. Median > Mean for noisy sensors**

Depth maps have outliers—pixels that read 0m or 100m due to reflections or occlusions. Taking the median of a 5×5 patch instead of a single pixel made the system dramatically more robust.

**3. Sample at the right location**

I don't sample depth at the bounding box center. I sample at the **cone base** (bottom 25% of the box). Cones are triangular—the center often contains background. The base is guaranteed to be the fat part of the cone.

**4. Terminal debugging is powerful**

Since I can't visualize custom ROS messages in RViz, I relied entirely on `ros2 topic echo` and logging. This forced me to think carefully about what data I actually needed to see.

## Phase 1 Complete

The Phase 1 perception pipeline is now operational:
- **Node 1a**: Camera publisher (30 Hz RGB + depth)
- **Node 2a**: YOLOv8 cone detector (30 FPS, 2D bounding boxes)
- **Node 3**: 3D localizer (pinhole back-projection + depth fusion)

**Output:** 3D cone positions in the camera frame (not yet validated against ground truth).

> **Important:** The current implementation uses **mock depth data** (estimated from vertical position in the image) and **estimated camera intrinsics** (not from actual calibration). The positions are based on geometric assumptions and will need validation with real depth sensors and proper camera calibration.

## What's Next

Phase 1 gives me 3D positions in the **camera frame**. But the path planner needs positions in the **vehicle frame** (`base_link`).

That requires coordinate transformations using ROS 2's `tf2` system. I also need to start thinking about Phase 2: adding LiDAR for redundancy and improved range.

But for now, Phase 1 works. The foundation is solid.

---

> **For comprehensive technical details on Node 3's code structure, implementation, troubleshooting, and design decisions, see the [Technical Deep-Dive: Node 3 - 3D Localizer](/fyp-perception-slam/techposts/node-3-localizer/).**
