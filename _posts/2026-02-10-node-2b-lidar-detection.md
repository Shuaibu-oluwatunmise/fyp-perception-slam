---
title: "Building Node 2b: LiDAR Cone Detection"
date: 2026-02-10 17:00:00 +0000
categories: [Week 4, Phase 2]
tags: [ros2, lidar, pcl, point-cloud-processing, clustering]
image:
  path: /assets/img/thumbnails/14.png
  alt: Node 2b Clustering Output
  header: false
pin: false
---

> **Technical Deep Dive:** For the full code explanation and architecture of Phase 2, see the [Technical Deep Dive](/fyp-perception-slam/techposts/phase-2-technical-deep-dive/).

Yesterday, I built Node 1b to generate raw point clouds. Today, I built **Node 2b**, the intelligence layer that finds cones in that chaos.

## The Approach: Geometric Clustering

Unlike Node 2a (Camera) which uses Deep Learning (YOLO), Node 2b uses **Classical Computer Vision** techniques. LiDAR data is geometric, so we use geometry to solve it.

I am using **DBSCAN** (Density-Based Spatial Clustering of Applications with Noise) from the `sklearn` library.

### The Pipeline

The `lidar_detector.py` script follows a 4-step process:

1.  **Distance Filtering:**
    *   Ignore points too close (< 0.5m) or too far (> 20m).
    
2.  **Ground Removal:**
    *   Remove all points with `Z < 0.05m`. This is a simple height threshold, though I plan to upgrade to RANSAC plane fitting later for robustness on slopes.

3.  **Clustering (DBSCAN):**
    *   Group points that are within `eps=0.3m` of each other.
    *   This naturally groups the ~75 points of a cone into a single object.

4.  **Geometric Validation:**
    *   Is the cluster the right size? 
    *   Height must be `0.2m - 0.4m`.
    *   Width must be `0.1m - 0.3m`.

## Results

The node publishes to `/detections/lidar`. The video below shows the clustering in action:

{% include embed/youtube.html id='0ai5mX7iISA' %}
*Node 2b in action: Converting raw point cloud clusters into discrete cone detections.* 

Since LiDAR has no color information, Node 2b currently guesses the class based on position (`Y > 0` is Left/Blue, `Y < 0` is Right/Yellow). This is a temporary heuristic. The **real** classification will happen tomorrow in **Node 3 (Fusion)**, where we combine this precise position with the camera's color data.

## Challenges

The main challenge was tuning `eps` for DBSCAN.
*   Too small -> One cone splits into two clusters.
*   Too large -> Two nearby cones merge into one blob.
*   **0.3m** seems to be the sweet spot for FS cones spaced 4m apart.
