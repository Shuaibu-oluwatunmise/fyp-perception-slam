---
title: "The Late Sensor Fusion Engine and The Hungarian Matrix"
date: 2026-03-06 18:00:00 +0000
categories: [Week 7, Phase 4]
tags: [sensor-fusion, hungarian-algorithm, perception, math, pipeline]
pin: false
image:
  path: /assets/img/thumbnails/32.png
  alt: Sensor Fusion Hungarian Matrix
  header: false
---
With the custom OpenCV debugger running and the hardware offsets aligning our sensors in 3D space, the structural geometry of the fusion was solved. When a LiDAR dot projected onto the camera lens, it landed exactly where it was supposed to.

But geometry isn't everything. During dynamic testing through tight corners, the pipeline was still occasionally dropping valid cones.

The issue wasn't the projection math—it was the decision algorithm. If two cones overlapped from the camera's perspective, the script struggled to pair the right LiDAR cluster with the right YOLO bounding box. It was relying on strict, rigid fallback checks (like a hard depth threshold) that broke the moment the visual scene got too cluttered.

Today, I completely tore out the core decision engine inside Node 3 and replaced it with a robust, mathematically optimal assignment matrix.

Here is the final result of that work—the complete perception stack running end-to-end across multiple track scenarios.

{% include embed/youtube.html id='u1XT78EHA4w' %}
_Full end-to-end run of the Late Sensor Fusion Engine across multiple track layouts._

---

## Improving the Logic

Previously, the 3D localizer's matching logic was fundamentally straightforward but inflexible. It would try to match the closest point, run a strict depth verification check, and if anything disagreed by more than 1.5 meters, it panicked and deleted the fusion entirely.

When you have 30 cones on screen, stacking up behind each other in a tight hairpin turn, rigid single-variable rules break down. I needed a system that could evaluate the entire scene at once and mathematically figure out the single best combination of all overlapping objects simultaneously.

Enter the **Hungarian Algorithm** (also known as the linear sum assignment problem solver).

## The 4-Factor Cost Matrix

Instead of deleting matching arrays outright when a single rule fails, every potential LiDAR-to-YOLO match is scored against a cost matrix. I built a scoring system that weighs four variables simultaneously to determine how "expensive" a match is. The goal is to minimise the total combined cost across the entire image.

Here is the exact math running for every cluster inside `localizer_3d.py`:

**1. Maximize Trapped Points (The Primary Rule)**
The more 3D laser points that successfully project *inside* a specific 2D YOLO bounding box, the lower the cost (`-1000.0 * float(points_in_box)`). If 15 laser returns land inside a blue box, that association is almost certainly correct.

**2. Minimize Focal Distance (The Secondary Rule)**
If two overlapping YOLO boxes both capture the same LiDAR points, we break the tie by Euclidean distance. Projecting closer to the centre of the YOLO box is cheaper (`avg_dist * 0.1`).

**3. Soft Depth Penalty (The Tertiary Rule)**
If the camera depth map disagrees with the LiDAR depth, I no longer delete the cone — I apply a soft penalty (`depth_diff * 50.0`) to account for cases where YOLO overdraws onto the background behind a distant cone.

**4. YOLO Confidence Reward (The Quaternary Rule)**
Higher detection confidence slightly reduces the pairing cost (`-confidence * 1.0`). If YOLO26s is 95% certain it's a yellow cone, that conviction is factored in.

The matrix is computed for every object on screen simultaneously and passed to `scipy.optimize.linear_sum_assignment` — the Hungarian solver — which returns the globally optimal assignment. Each LiDAR cluster is matched to at most one YOLO box and vice versa, with no greedy local decisions that can cascade into global errors.

## The Final Pipeline in Action

This is what Phase 4 looks like when every upgrade from this week comes together.

3D volumetric filtering rejecting the walls. Inverse-square optical grading filtering out low-reflectivity debris. K-Means recovering merged close-proximity cones. YOLO26s extending detection range. And the Hungarian assignment matrix resolving the final LiDAR-to-camera matching across the full scene.

The perception architecture is structurally complete and producing stable detections across the track scenarios we've tested.

With the data geometry and optical filtering finalized, the next step is moving into formal structured testing. Week 8 will focus entirely on evaluating the pipeline—measuring latency and calculating the exact positional error of the localized cones against ground truth.

Week 7 is over, and the Perception Development Phase is complete.