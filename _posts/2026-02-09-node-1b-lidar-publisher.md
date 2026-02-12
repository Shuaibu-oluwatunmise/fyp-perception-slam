---
title: "Building Node 1b: The LiDAR Publisher"
date: 2026-02-09 17:00:00 +0000
categories: [Week 4, Phase 2]
tags: [ros2, lidar, point-cloud, velodyne, implementation]
pin: false
---

> **Technical Deep Dive:** For the full code explanation and architecture of Phase 2, see the [Technical Deep Dive](/fyp-perception-slam/techposts/phase-2-technical-deep-dive/).

Phase 1 (Camera) is done. Today marks the start of **Phase 2: LiDAR Integration**.

The goal is to bring the "Second Eye" into the system. While the camera gives us color (cone class), the LiDAR gives us precise distance (geometry).

## The Data Source: Node 1b

**Node 1b** acts as the data acquisition layer for our laser-based perception. It provides a unified interface (`/lidar/pointcloud`) regardless of whether we are running on:
1.  **Mock Mode:** Generating synthetic cone clusters for testing.
2.  **Velodyne VLP-16:** The real hardware driver.
3.  **CarMaker:** High-fidelity simulation.

Since I am currently working without the full rig, I implemented the **Mock Mode** to generate realistic test data.

## Implementation Details

The node is written in **Python** and uses the `struct` library to manually pack binary `PointCloud2` messages. This avoids heavy dependencies like `pcl_conversions` which can be tricky to set up.

### The Mock Generator
Instead of just publishing single points, Node 1b generates **clusters** of ~75 points per cone. 

```python
# Generates a cone-shaped cluster
for cone_pos in self.cone_positions:
    cone_points = self._generate_cone_cluster(x, y, z)
    points.extend(cone_points)
```

It places:
*   **Blue Cones** on the Left (`Y ≈ +2.5m`)
*   **Yellow Cones** on the Right (`Y ≈ -2.5m`)
*   **Ground Plane** noise at `Z ≈ 0`

This forces the downstream nodes to actually solve the clustering problem, rather than being spoon-fed perfect centroids.

## Visualization

Here is the output visualized in RViz. You can see the "fuzzy" clusters representing cones and the scattered points representing the ground.

{% include embed/youtube.html id='tNwB5HGt-ug' %}
*Node 1b Output: Raw PointCloud2 visualization in RViz showing cone clusters and ground noise.*

## Why `PointCloud2`?

I chose `sensor_msgs/PointCloud2` over `LaserScan` because Formula Student cars often pitch down during braking. A 2D laser scan might hit the ground or fly over the cones. A 3D point cloud ensures we see the cones regardless of vehicle dynamics.

Tomorrow (Feb 10), I build **Node 2b** to make sense of this raw data.
