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

Instead of deleting matching arrays outright when a single rule fails, every potential LiDAR-to-YOLO match is now heavily scrutinized against a structural cost matrix. I built a dynamic scoring system that weighs four different variables at once to determine how "expensive" a match is. The goal is to minimize the total combined cost across the entire image.

Here is the exact math running for every cluster inside `localizer_3d.py`:

**1. Maximize Trapped Points (The Primary Rule)**
The more 3D laser points that successfully project *inside* a specific 2D YOLO bounding box, the vastly cheaper the cost (`-1000.0 * float(points_in_box)`). I want to maximize the overlapping physical volume heavily. If 15 laser returns land in a blue box, that is undoubtedly a blue cone.

**2. Minimize Focal Distance (The Secondary Rule)**
If two overlapping YOLO boxes both capture the same LiDAR points, how do we break the tie? We check Euclidean distance. Projecting closer to the exact mathematical center of the YOLO box is cheaper (`avg_dist * 0.1`).

**3. Soft Depth Penalty (The Tertiary Rule)**
If the camera depth map heavily disagrees with the LiDAR depth, I don't delete the cone anymore. I just apply a soft sliding mathematical penalty (`depth_diff * 50.0`) to account for times when YOLO overdraws and accidentally measures the grass behind a distant cone.

**4. YOLO Confidence Reward (The Quaternary Rule)**
Finally, higher neural network confidence slightly reduces the pairing cost (`-confidence * 1.0`). If YOLO26s is 95% certain it's a yellow cone, we reward that conviction.

The matrix calculates this formulation for every single object on screen simultaneously. It then feeds the massive grid of numbers into the Hungarian solver, which churns out the globally optimal, lowest-cost combination for all overlapping cones in a fraction of a millisecond.

No more flickering colours. No more dropped data. Just pure, deterministic mathematics.

## The Final Pipeline in Action

This is exactly what Phase 4 looks like when every upgrade from this week comes together.

True 3D volumetric filtering rejecting the walls. Inverse-Square optical grading ignoring the debris. K-Means slicing the hairpin merges. YOLO26s spotting distant cones. And finally, the Hungarian assignment matrix weaving the two distinct sensor streams together into a single, flawless understanding of the track.

The perception architecture is now structurally complete and actively tracking across the track scenarios we've tested. 

With the data geometry and optical filtering finalized, the next step is moving into formal structured testing. Week 8 will focus entirely on evaluating the pipeline—measuring latency and calculating the exact positional error of the localized cones against ground truth.

Week 7 is over, and the Perception Development Phase is complete.
