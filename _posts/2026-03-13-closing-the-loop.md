---
title: "Closing the Loop: Frames, Projection, and the Three Outcomes of Fusion"
date: 2026-03-13 18:00:00 +0000
categories: [Week 8, Technical Deep Dive]
tags: [perception, fusion, lidar, camera, tf2, hungarian, interactive]
pin: false
image:
  path: /assets/img/thumbnails/34.png
  alt: Sensor Fusion
  header: false
---

Monday we covered why camera depth fails at distance. Wednesday we saw how LiDAR builds a clean point cloud. Today is where those two streams meet — and how we resolve them into one authoritative list of 3D cone positions.

## The Frame Problem

Before we can combine anything, we have to address a fundamental issue: the LiDAR and the camera don't share a coordinate system. LiDAR points arrive in `Lidar_F` — a frame centred on the sensor, X forward, Y left, Z up. Camera detections live in pixel space, originally derived from `Obj_F` — the camera's optical frame, where Z points forward along the lens axis, X is right, and Y is down.

The car's navigation output expects positions in `Fr1A` — the vehicle body frame.

**TF2** is the ROS transform tree that maintains all of these relationships dynamically. At every fusion step we query it for the exact transform at that message's timestamp:

```python
dyn_t, dyn_R = self.lookup_tf_full('Lidar_F', 'Obj_F', stamp=stamp)
```

This returns the rotation and translation needed to move a point from the LiDAR frame into the camera frame at that exact instant — accounting for any relative motion between sensors that may have occurred.

## Projecting Into Image Space

Once we have a LiDAR cluster centroid in the camera frame, we apply the standard pinhole projection:

```python
# Vehicle frame → camera optical frame
cam_x, cam_y, cam_z = lidar_to_camera(pt, dyn_t, dyn_R)

# 3D → 2D pixel
u = fx * (cam_x / cam_z) + cx
v = fy * (cam_y / cam_z) + cy
```

This gives us a pixel coordinate `(u, v)` — where this LiDAR cluster *would appear* in the camera image. The goal is then to check: does any YOLO bounding box contain this pixel?

This is the same geometry from the March 9th post, just run in reverse purpose — we're not asking "what's the depth behind this pixel" but "which pixel does this 3D point project onto."

## The Three Outcomes

The matching uses the **Hungarian algorithm** to find the optimal global assignment between LiDAR clusters and YOLO boxes. The cost function rewards clusters whose projected points land deep inside bounding boxes and penalises depth disagreement with the camera's depth map.

Every cone ends up in exactly one of three states:

| Outcome | Condition | Position | Colour | Confidence |
|---|---|---|---|---|
| **Fused** | LiDAR projection inside YOLO box | LiDAR 3D | YOLO label | 1.0 |
| **LiDAR-only** | Cluster with no matching box | LiDAR 3D | `unknown_cone` | 0.9 |
| **Dropped** | YOLO box with no LiDAR hit | — | — | 0 |

Camera-only detections are dropped entirely. If the camera sees something but LiDAR doesn't confirm it, we don't publish it — no depth source we trust well enough.

The widget below makes this concrete. The **left panel** is a top-down view of the LiDAR frame — you can see two cone positions and the camera's field of view. The **right panel** is the camera image with two YOLO boxes already present. Use the sliders to adjust each cone's depth (X, how far forward) and lateral position (Y, how far left or right). Watch the projected crosshair move across the camera image in real time — and see how the fusion outcome changes the moment it enters or exits a bounding box:

<iframe src="{{ site.baseurl }}/interactive_widgets/fusion_widget.html" width="100%" height="620px" scrolling="no" frameborder="0" style="border:none; border-radius:12px; margin: 20px 0; box-shadow: 0 10px 30px rgba(0,0,0,0.5);"></iframe>

One practical wrinkle: our CarMaker camera runs at ~3.3 Hz and the LiDAR at ~1 Hz. To synchronise them at all, the `ApproximateTimeSynchronizer` uses a **1.5 second tolerance window**. Any tighter and virtually every pair is dropped before fusion even starts. This is a hardware constraint of the simulation environment — on a real vehicle both sensors would run considerably faster.

## The Output

Every fused or LiDAR-only cone is published on `/cones` as a `Cone3D` message with `(x, y, z)` in the `Fr1A` vehicle frame. The navigation stack reads this topic and sees: here are the cones, in my own coordinate frame, ready to plan around.

That's the full pipeline — from a spinning laser and a colour camera, down to a clean list of 3D positions. Next week moves into formal evaluation: does it actually work at the numbers the FSAI objective requires?
