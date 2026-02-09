---
title: "Planning Node 3: From 2D Detections to 3D Positions"
date: 2026-02-05 09:00:00 +0000
categories: [Planning, Week 3]
tags: [ros2, 3d-localization, camera-intrinsics, planning, node-3]
pin: false
---

Node 2a is working. It's detecting cones and publishing bounding boxes at 30 FPS. But there's a problem.

The path planner doesn't care about pixels. It needs real-world positions—meters, not pixel coordinates. **"There's a cone at pixel (320, 240)"** is useless for navigation. **"There's a cone 8 meters ahead, 2 meters to the right"** is what we need.

Today, I spent time researching how to bridge that gap. How do we convert 2D detections `(u, v)` into 3D positions `(X, Y, Z)`? This is the **localization** step in sensor fusion—taking semantic information (what the object is) and adding metric information (where it is).

The architecture I'm using follows a **Late Fusion** (decision fusion) approach [^1]: Node 2a makes detection decisions first, then Node 3 adds spatial information. This is more robust than early fusion because failures in one stage don't cascade through the entire pipeline.

## The Core Challenge

Node 2a outputs bounding boxes in image space:
- Center: `(u=320, v=240)`
- Class: `blue_cone`
- Confidence: `0.92`

But the path planner needs:
- Position: `(X=2.5m, Y=0.3m, Z=8.0m)`
- Frame: `camera_frame`

The missing piece? **Depth**. Without knowing how far away the cone is, we can't convert pixels to meters.

## The Pinhole Camera Model

After digging through computer vision textbooks and ROS 2 documentation, I found the mathematical foundation: the **Pinhole Camera Model**.

The forward projection (3D world → 2D image) is:

```
u = fx * (X/Z) + cx
v = fy * (Y/Z) + cy
```

Where:
- `u, v` = Pixel coordinates
- `X, Y, Z` = 3D position in camera frame (meters)
- `fx, fy` = Focal length (pixels) - the "zoom" of the lens
- `cx, cy` = Optical center (pixels) - usually the image center

Node 3 needs to **invert** this. We know `u, v` (from Node 2a) and `Z` (from depth data). We need to solve for `X` and `Y`:

```
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
```

This is the core math. Everything else is just handling edge cases and noise.

## Camera Intrinsics

The parameters `fx, fy, cx, cy` are called **camera intrinsics**. They're unique to each camera and lens combination.

For the video footage I'm using (from the FSAI Chalmers team), I'll need to either:
1. Find the published intrinsics (if available)
2. Estimate them from the video metadata
3. Use typical values for similar cameras

This is something I'll need to verify carefully—wrong intrinsics will cause cones to appear in completely wrong positions.

## The Depth Problem

The pinhole model requires depth (`Z`). Node 1a publishes `/camera/depth`, but it's currently mock data (all zeros or a fixed value).

For Phase 1 testing, I'll need to create a **fake depth publisher** that generates plausible depth values. The strategy:
- Assume cones are on the ground plane
- Estimate depth based on vertical position in the image (lower in image = closer)
- Add some noise to simulate real sensor behavior

This won't be perfect, but it'll let me test the pipeline. When we get real depth data (from a ZED camera or LiDAR), the math stays the same—just better input data.

## Time Synchronization

This is the part that worries me most.

Node 2a adds ~30ms of inference latency. The depth map doesn't wait for inference—it publishes immediately. When Node 3 receives a detection message, it needs to match it with the **correct** depth map from the same frame.

The solution: **ApproximateTimeSynchronizer** from `message_filters`.

```python
self.ts = message_filters.ApproximateTimeSynchronizer(
    [self.detections_sub, self.depth_sub],
    queue_size=10,
    slop=0.1  # 100ms tolerance
)
```

This holds depth maps in a buffer and only calls the processing callback when it finds a detection message with a matching timestamp (within 100ms).

If I get this wrong, Node 3 will match cones from Frame N with depth data from Frame N+1, and positions will be completely off.

## Depth Sampling Strategy

Depth maps are noisy. They have:
- **Flying pixels** at object edges (interpolation errors)
- **Occlusion shadows** (black regions where depth is unknown)
- **Reflections** (shiny surfaces returning bad data)

I can't just sample a single pixel at the bounding box center. I need a robust strategy.

The plan: **Median patch sampling**
1. Sample a 5×5 pixel patch around the cone base (not center—base is more reliable)
2. Filter out invalid depth values (zeros, infinities)
3. Take the **median** (not mean—median ignores outliers)

This should handle noise much better than naive single-pixel sampling.

## Node 3 Architecture

Here's the plan:

**Inputs:**
- `/detections` (from Node 2a) - 2D bounding boxes
- `/camera/depth` (from Node 1a) - Depth map

**Processing:**
1. Synchronize detection and depth messages by timestamp
2. For each detection:
   - Extract pixel coordinates `(u, v)`
   - Sample depth `Z` using median patch filter
   - Apply pinhole back-projection to get `(X, Y, Z)`
3. Construct `Cone3D` messages

**Outputs:**
- `/cones_camera_frame` - 3D cone positions in camera frame

**Design principle:** Keep Node 3 stateless. It doesn't track cones over time or maintain a map. It just solves the geometry for each frame independently.

## What Could Go Wrong?

I'm anticipating a few issues:

1. **Wrong intrinsics** → Cones appear in wrong positions (systematic error)
2. **Bad depth data** → Cones jump around frame-to-frame (noise)
3. **Synchronization failure** → Very low output rate or misaligned frames
4. **Sampling at wrong location** → Depth samples hit background instead of cone

I'll need to monitor the terminal output carefully—checking the published 3D positions with `ros2 topic echo` and adding logging to catch these issues early.

## Next Steps

Tomorrow, I'll implement Node 3. The math is straightforward—it's the edge cases and debugging that will take time.

If I can get this working, I'll have a complete Phase 1 pipeline: **Camera → Detection → 3D Localization**. That's the foundation for everything else.

---

[^1]: J. Fayyad, M. A. Jaradat, D. Gruyer, and H. Najjaran, "Deep Learning Sensor Fusion for Autonomous Vehicle Perception and Localization: A Review," *Sensors*, vol. 20, no. 15, p. 4220, 2020. The paper categorizes sensor fusion into early fusion (data-level), halfway fusion (feature-level), and late fusion (decision-level), where "multiple classifiers are used to generate decisions that are then combined to form a final decision."
