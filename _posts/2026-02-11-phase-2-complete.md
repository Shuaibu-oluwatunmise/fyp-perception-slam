---
title: "Phase 2 Complete: Sensor Fusion"
date: 2026-02-11 17:00:00 +0000
categories: [Week 4, Phase 2]
tags: [ros2, sensor-fusion, camera-lidar-fusion, phase-2-complete]
pin: false
---

> **Technical Deep Dive:** For the full code explanation and architecture of Phase 2, see the [Technical Deep Dive](/fyp-perception-slam/techposts/phase-2-technical-deep-dive/).

Today marks the end of the **Phase 2 Implementation Sprint**. I have upgraded **Node 3 (3D Localizer)** to support **Sensor Fusion**.

## The Fusion Engine

We now have two inputs:
1.  **Camera (Node 2a):** Good at "What is it?" (Color) but bad at "Where is it?" (Depth).
2.  **LiDAR (Node 2b):** Bad at "What is it?" (No Color) at "Where is it?" (Accurate Depth).

The new `localizer_3d.py` fuses them using a **Weighted Average Strategy**.

### How it works

1.  **Transform:** The LiDAR points are transformed into the Camera's Optical Frame (`X`, `Y`, `Z`) using the extrinsic calibration matrix.
2.  **Associate:** We use a Nearest Neighbor search. If a LiDAR cone is within 1.0m of a projected Camera cone, they are matched.
3.  **Merge:**
    *   **Position:** Calculated as `0.1 * Camera + 0.9 * LiDAR`. We trust the LiDAR much more for position.
    *   **Class:** Taken from the Camera (since LiDAR is colorblind).

### Handling Unmatched Cones

*   **Camera-Only:** We keep them. Sometimes the LiDAR misses things (black cones don't reflect well), or the object is out of LiDAR range.
*   **LiDAR-Only:** We add them as "Unknown Color". This ensures we don't crash into invisible obstacles just because the camera didn't see them.

## Visualization

In RViz, I used color coding to debug the fusion:
*   **GREEN:** Fused (Both sensors saw it).
*   **BLUE:** Camera-Only.
*   **YELLOW:** LiDAR-Only.

{% include embed/youtube.html id='qYOcxFLtRaY' %}
*Sensor Fusion in action: Green markers show successful fusion of Camera and LiDAR data.*

## The Reality Check

The Fusion Engine seems to be functioning. Green boxes appear where they should.

Is it ready for the car? I'm not so sure.

