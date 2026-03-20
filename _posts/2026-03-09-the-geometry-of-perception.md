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

Instead of just pasting code snippets, I’ve built interactive playgrounds directly into this post. Let's start with the fundamental geometry of how the car sees the world.

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

The original iteration of this project tried to estimate 3D cone positions purely using the camera's depth map, and then match those full 3D coordinates against the LiDAR coordinates. 

This failed spectacularly, and it comes down to standard pixel error. The mathematical formula for lateral error based solely on depth is:
`lateral_error = pixel_error × Z / fx`

Below is an interactive widget running that exact math engine. 
**Try dragging the Depth slider out past 20 metres and slightly increase the Pixel Error from YOLO's bounding box.** Watch what happens to the theoretical lateral accuracy.

<iframe src="{{ site.baseurl }}/interactive_widgets/camera_widget.html" width="100%" height="980px" scrolling="no" frameborder="0" style="border:none; border-radius:12px; margin: 20px 0; box-shadow: 0 10px 30px rgba(0,0,0,0.5); overflow:hidden;"></iframe>

### The Conclusion: Image-Space Matching
As you can see in the widget, a tiny 5-pixel jitter in YOLO's bounding box at 25 metres deep creates a **0.39m lateral error**. That completely busts the FSAI ≤0.20m boundary requirement. The camera simply cannot accurately place objects physically.

**The Solution:** Instead of trying to guess where the camera's pixels are in the 3D world, we reverse the math. We project the highly accurate 3D LiDAR points *into* the 2D image. 
- The **LiDAR** gives us the exact 3D position.
- The **Camera image** acts purely as a 2D sorting mechanism to give those points their semantic colour.

By projecting LiDAR down into the image space, camera depth is never used for physical placement, securing millimetre accuracy at extreme range.

### The Real-World Application
This is essentially how we do this:

![Image-Space Matching Architecture](/assets/img/blog/2026-03-09/debug.png)
