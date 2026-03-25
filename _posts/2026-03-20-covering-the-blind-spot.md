---
title: "Covering the Blind Spot"
date: 2026-03-20 18:00:00 +0000
categories: [Week 9, SLAM]
tags: [slam, mapping, camera, lidar, localisation, fallback, ros2]
image:
  path: /assets/img/thumbnails/38.png
  alt: Camera-Only Fallback
  header: false
pin: false
---

Back in the [geometry of perception post](/fyp-perception-slam/the-geometry-of-perception/), there was a note flagged as future work:

> At close range — say under 5 m — there is a real argument for using camera-derived positions to recover cones that LiDAR may have missed, where sparse point returns and ground-plane ambiguity are most problematic.

That future work got implemented. This post is what it turned into.

---

## The Problem

As the car approaches a cone, LiDAR point returns on it become sparser. Ground-plane filtering starts clipping the base returns. At some point — and in CarMaker this happens at a larger range than the 0.2 m minimum we configured — the cluster disappears from LiDAR detection entirely.

The camera still sees it. The cone is large in the frame, the bounding box is clean, YOLO is confident. But the fusion node requires a LiDAR cluster to anchor the 3D position — a YOLO detection with no LiDAR hit gets dropped. So at the exact moment the car is closest to a cone and has the clearest visual on it, the cone disappears from the output.

This is not catastrophic — most cones are already confirmed in the map before the car gets that close, so they stay there. But it is a gap in coverage, and closing it makes the system more robust. A cone that was only ever seen at close range would never get confirmed at all without a fallback.

---

## The Solution

Camera-only fallback fills the gap. After the Hungarian assignment runs and matched pairs are resolved, any YOLO detection that went unmatched — no LiDAR cluster was assigned to it — becomes a candidate for fallback localisation.

The depth image (`/camera/depth`, 32-bit float in metres) is already being subscribed to in the fusion node. For each unmatched detection, a central region of the bounding box is sampled and the **10th percentile** of valid depth values is used. The 10th percentile picks the closest valid surface in the region — the cone face, not the background behind it.

That depth value, combined with the bounding box centre pixel and the camera intrinsics, back-projects into a 3D position in the camera frame:

```
x_cam = (px - cx) * depth / fx
y_cam = (py - cy) * depth / fy
z_cam = depth
```

This is then converted to the vehicle body frame and transformed into `Fr1A` using the same TF2 lookup used everywhere else in the pipeline. The result is a `Cone3D` message published with `source = 'camera_only'`.

---

## Why It Does Not Corrupt the Map

The obvious concern: depth back-projection from a camera is noisier than LiDAR range measurement. If camera-only positions are allowed to update the map freely, they could drift accurate LiDAR-confirmed positions.

This is exactly what the source-aware architecture described in Wednesday's post was designed to prevent. The priority hierarchy is:

```
camera_only  <  lidar  <  fused
```

A cone that has been confirmed as `fused` or `lidar` will never have its stored position updated by a `camera_only` observation. The accurate position is frozen. The camera-only observation is effectively used only for cones that have not yet been seen by LiDAR — typically cones first encountered at very close range.

The global mapper also runs a **periodic self-cleanup** every 3 seconds. It scans all stored cone pairs and merges any `camera_only` entry that is within 2 m of a `fused` or `lidar` entry — keeping the trusted position, removing the camera-only duplicate. This handles cases where a camera-only cone was created slightly before the local mapper could suppress it as a duplicate of an already-accurate entry. Same-source pairs are never merged, since two real adjacent cones can legitimately be close together.

---

## The Guard Against Duplicates

Before a camera-only cone is published, it is checked against all existing cones in the current frame's output. If any `fused` or `lidar` cone is already within 0.5 m, the candidate is dropped — it is a duplicate, and the accurate version already exists. This upstream check prevents the mapper from even seeing a camera-only duplicate in the first place, and the global mapper's periodic cleanup catches anything that slips through.

---

## It Is Optional

The whole fallback is controlled by a single launch parameter:

```bash
ros2 launch fsai_perception perception.launch.py source:=carmaker camera_fallback_enabled:=true
```

By default it is off. The reasons for keeping it optional:

- On tight track sections like Skidpad, cones are closely packed and the 4 m depth threshold and 0.5 m separation guard are not sufficient to reliably distinguish adjacent cones from each other. Fallback is most reliable when cones are well-spaced and roughly ahead of the camera.
- Depth image quality matters. In CarMaker simulation the depth image is clean. On real hardware, depth near edges and at close range can be noisy.
- For Trackdrive and Acceleration where the geometry is straightforward, it works well and fills a real gap.

---

## What It Looks Like

![Camera-only fallback in RViz — large orange cones localised despite LiDAR blind spot](/assets/img/blog/2026-03-20/camera-only.png)

This is the system running in RViz. The red LiDAR scan lines show the sensor sweep — you can see the coverage gap close to the car. The large orange cones near the vehicle are localised and showing as markers in the map despite being inside that gap. Those are camera-only cones. The camera saw them, back-projected their depth, and they made it into the map cleanly.

That gap between what LiDAR can see and what the camera can see is small but real. This closes it.
