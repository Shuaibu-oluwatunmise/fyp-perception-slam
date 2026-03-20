---
title: "The Geometry of Perception: Why Pinhole Math is Better Than Camera Depth"
date: 2026-03-09 18:00:00 +0000
categories: [Week 8, Technical Deep Dive]
tags: [perception, geometry, pinhole-camera, interactive, math]
pin: false
image:
  path: /assets/img/thumbnails/33.png
  alt: Geometry of Perception
  header: false
---
This week, before diving into the formal simulation test results, I want to take a step back and look at the underlying mathematics that power the FSAI perception fusion engine.

Instead of just pasting code snippets, I've built interactive playgrounds directly into this post. Let's start with the fundamental geometry of how the car sees the world.

---

## 1. The Pinhole Camera Model

A pinhole camera is the simplest mathematical model of how a lens maps the 3D world into a flat 2D image. By understanding that light from the world passes through a single point (the **optical centre**), we can define a strict geometric relationship between objects on the track and pixels on the screen.

The projection formula is elegant:

```
u = fx × (X / Z) + cx
v = fy × (Y / Z) + cy
```

- `X, Y, Z` = 3D position of an object in metres.
- `u, v` = Pixel column and row in the final image.
- `fx, fy` = The focal lengths (the "zoom" of the lens).
- `cx, cy` = The principal point (the exact centre of the image).

In CarMaker, our camera runs at exactly a 90° horizontal Field of View (FOV) at 640×480 resolution. By geometry, `fx = 320` and `cx = 320`.

---

## 2. The Axis Permutation Trap

Before we can use the pinhole formula, there is a giant trap: **Coordinate frames.**

The vehicle body (and the LiDAR) operates in a standard vehicle frame: `X` points forward, `Y` points left, and `Z` points up.
But optical lenses follow the computer vision standard (`Obj_F` in CarMaker): `X` points right, `Y` points down, and `Z` points forward into the screen (depth).

If you feed LiDAR points directly into the projection formula, the math breaks. Every pixel projects wildly off-screen. Inside `localizer_3d.py`, we execute a mandatory axis swap immediately after applying the rigid physical TF2 transform:

```python
xformed = dyn_R.apply(pt) + dyn_t   # 1. TF2 rigid transform

# 2. Axis swap: vehicle → optical
# veh_X (forward) → opt_Z (depth)
# veh_Y (left)    → opt_X (right, negated)
# veh_Z (up)      → opt_Y (down, negated)
return -xformed[1], -xformed[2], xformed[0]
```

---

## 3. Why We Rely on LiDAR for 3D Matching

The obvious first instinct when you have a camera is to use it for everything — including depth. If YOLO detects a cone at pixel `(u, v)`, you can invert the projection formula to estimate where that cone sits in 3D space. The pipeline would then match those camera-derived 3D coordinates against the LiDAR point cloud.

Once the axis swap from Section 2 is handled correctly, the projection itself is accurate — the problem is what happens when you try to invert it.

This failed spectacularly, and it comes down to one unavoidable problem: YOLO's bounding box is never pixel-perfect. There is always some jitter in where the centre of the box lands. And when you invert the projection to recover a 3D position, that pixel error gets amplified by depth:

`lateral_error = pixel_error × Z / fx`

The widget below makes this concrete. Two cones sit at fixed depths — 8 m and 16 m. Their true positions are shown as dashed ghost squares in the top-down view. The solid squares are what the camera *estimates* based on the bbox centre. At 1 px of jitter both estimates are nearly perfect. **Drag the pixel error slider up** and watch the estimates drift — the ghost stays fixed while the solid square pulls away from it. The far cone drifts twice as fast for the same jitter.

<iframe src="{{ site.baseurl }}/interactive_widgets/camera_widget.html" width="100%" height="480px" scrolling="no" frameborder="0" style="border:none; border-radius:12px; margin: 20px 0; box-shadow: 0 10px 30px rgba(0,0,0,0.5); overflow:hidden;"></iframe>

### The Conclusion: Image-Space Matching

Just 4 pixels of bbox jitter puts the far cone at exactly the FSAI ≤0.20 m boundary. At 5 px it blows past it — and real-world YOLO jitter is rarely as clean as 5 px. The ghost never moves; the estimate does. That gap is the problem.

**The Solution:** Instead of trying to guess where the camera's pixels are in the 3D world, we reverse the math entirely. We project the highly accurate 3D LiDAR points *into* the 2D image.

- The **LiDAR** gives us the exact 3D position — no depth estimation, no error amplification.
- The **Camera image** acts purely as a 2D lookup to give those points their semantic colour.

By projecting LiDAR into image space rather than the other way around, camera depth is never used for physical placement, and the ghost and the estimate are always the same point.

> **A note on close-range detection:** This is not to say camera depth is useless. At close range — say under 5 m — the bbox is large, jitter is proportionally small, and the lateral error stays well within the 0.20 m threshold. There is a real argument for using camera-derived positions to recover cones that LiDAR may have missed at close range, where sparse point returns and ground-plane ambiguity are most problematic. Fusing camera depth estimates as a fallback for close-range LiDAR misses is a natural next step. It is flagged here as future work.