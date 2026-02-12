---
title: "The Missing Link: Why Phase 2 Wasn't Finished"
date: 2026-02-12 01:00:00 +0000
categories: [Week 4, Phase 2]
tags: [ros2, tf2, coordinate-systems, slam-integration, node-4]
image:
  path: /assets/img/thumbnails/16.png
  alt: Frame Transformation Result
  header: false
pin: false
---

> **Technical Deep Dive:** For the full code explanation and architecture of Phase 2, see the [Technical Deep Dive](/fyp-perception-slam/techposts/phase-2-technical-deep-dive/).

I thought I finished Phase 2 yesterday. I was wrong.

We successfully fused Camera and LiDAR data. We had functional cone detection. In my mind, the perception system was "Done".

But today, when I looked at integrating this with the SLAM system for Phase 3, I realized we were missing one critical aspect. We were speaking the wrong language.

## The Problem: "Camera-Speak" vs "Car-Speak"

Up until now, our perception system has been selfish. It reports cone positions relative to the *camera lens* (Optical Frame):
*   `Z` is Forward (Depth)
*   `X` is Right
*   `Y` is Down

But the rest of the car—the Path Planner and SLAM—expects standard **ROS REP-103** coordinates (Vehicle Frame):
*   `X` is Forward
*   `Y` is Left
*   `Z` is Up

If we send "Camera-Speak" to the Planner, the car will think a cone 10 meters ahead is actually 10 meters *above* it.

{% include embed/youtube.html id='-NotuvpKzgA' %}
*The Solution in Action: Node 4 transforming cones into the correct vehicle frame.*

## The Solution: Node 4

**Node 4 (Frame Transformer)** acts as the **Hardware Abstraction Layer**. It takes the `Cone3DArray` from Node 3 and applies two transformations:

1.  **Axis Re-mapping:** Swapping `Z` -> `X`, `-X` -> `Y`, `-Y` -> `Z`.
2.  **Physical Mounting:** Accounting for the fact that the camera is mounted 1.6m in front of the rear axle and 0.8m off the ground.

## Implementation

The transformation is mathematically simple but critical. We use a standard rotation matrix:

```
[ X_car ]   [  0  0  1 ]   [ X_opt ]
[ Y_car ] = [ -1  0  0 ] * [ Y_opt ]
[ Z_car ]   [  0 -1  0 ]   [ Z_opt ]
```

And then add the translation vector from `camera_mounting.yaml`.

## Why Separate It?

Why not just do this math inside Node 3?
Because **Node 3 shouldn't care what car it's on.**

By keeping the vehicle-specific mounting data in Node 4, we can take the exact same perception system (Nodes 1-3) and put it on a different car (e.g., the Driverless Vehicle vs the Electric Vehicle) just by changing Node 4's config file.

## Final Result

We now have cones publishing to `/cones` in the `base_link` frame. This is theoretically what the SLAM system needs to build a map. `Node 4` is the bridge that attempts to unify the system.

Tomorrow, we verify everything in **Simulation**.
