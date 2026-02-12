---
title: "Technical Deep Dive: Phase 2 Architecture & Implementation"
date: 2026-02-12 01:30:00 +0000
categories: [Week 4, Technical Deep Dive]
tags: [ros2, architecture, implementation, code-analysis, fusion, pcl, dbscan]
pin: true
---

This post is the **definitive technical reference** for the Phase 2 Perception Subsystem (LiDAR + Sensor Fusion). 

While the daily blog posts cover the *journey*, this document covers the *implementation details*, *design rationale*, and *code structure* that power the system.

## Architecture Overview

Phase 2 introduces three new nodes to the pipeline, transforming it from a simple Camera logger into a multi-sensor fusion engine.

| Node | Role | Key Tech | Rate |
| :--- | :--- | :--- | :--- |
| **Node 1b** | LiDAR Data Acquisition | Python `struct`, Mock Generation | 10 Hz |
| **Node 2b** | Geometric Clustering | `sklearn.DBSCAN`, Numpy | 10 Hz |
| **Node 3 (Fused)** | Sensor Fusion | Weighted Average, Extrinsics | 30 Hz |
| **Node 4** | Frame Transformation | TF2 Logic, Matrix Math | 30 Hz |

---

## 1. Node 1b: The LiDAR Interface

**Role:** Abstraction Layer for Hardware.
**Source:** `lidar_publisher.py`

### The Challenge: Binary Packing in Python
We publish `sensor_msgs/PointCloud2`. Unlike simple formats, this is a binary blob. Most people use C++ or `pcl_conversions`, but those are heavy dependencies. I chose to implement a **manual binary packer** using Python's standard `struct` library.

**Implementation:**
We pack `x`, `y`, `z` floats into 12 bytes per point.

```python
import struct

# Create the binary blob
cloud_data = []
for point in points:
    # 'fff' = three floats (4 bytes each)
    packed = struct.pack('fff', point[0], point[1], point[2])
    cloud_data.append(packed)

# Assign to ROS message
msg.data = b''.join(cloud_data)
```

### Mock Data Strategy
To test downstream nodes without a real sensor, Node 1b generates "fuzzy" cone clusters.
*   **Cone:** ~75 points distributed in a cone shape.
*   **Noise:** Gaussian jitter added to every point.
*   **Ground:** Sparse points at `Z ≈ 0`.

This forces **Node 2b** to actually solve the clustering problem, rather than just receiving perfect centroids.

---

## 2. Node 2b: Geometric Clustering

**Role:** Finding objects in the chaos.
**Source:** `lidar_detector.py`

### The Algorithm: DBSCAN
We use **Density-Based Spatial Clustering of Applications with Noise (DBSCAN)** from the `sklearn` library. This was chosen over K-Means because:
1.  **No fixed K:** We don't know how many cones are visible.
2.  **Noise handling:** It ignores sporadic "flying pixel" noise.

**Configuration:**
*   `eps = 0.3m`: The maximum distance between neighbors. (Derived from cone width ~30cm).
*   `min_samples = 10`: Minimum points to define a cluster.

### Geometric Validation
Once points are clustered, we validate them using simple geometric heuristics. A cluster is only a cone if:

```python
height = max_z - min_z
width = max(width_x, width_y)

# Standard FS Cone dimensions with tolerance
if 0.2 < height < 0.4 and 0.1 < width < 0.3:
    return True # It's a cone
return False # It's a rock / wall / leg
```

---

## 3. Node 3: The Fusion Engine

**Role:** Combining Camera (Color) + LiDAR (Position).
**Source:** `localizer_3d.py`

### Coordinate Alignment (Extrinsics)
Before fusion, we must map LiDAR points into the Camera's coordinate system.
*   **LiDAR Frame:** origin at sensor center.
*   **Camera Optical Frame:** origin at lens.

We apply the Extrinsic Matrix:
`P_cam = R_ext * P_lidar + T_ext`

### Data Association (Nearest Neighbor)
We match detections using Euclidean distance in 3D space.

```python
# Simple Nearest Neighbor Logic
match_found = False
for lid_cone in lidar_cones:
    dist = calculate_distance(cam_cone, lid_cone)
    
    if dist < 1.0: # 1 meter threshold
        fused_cone = merge(cam_cone, lid_cone)
        match_found = True
        break
```

**Why 3D matching?**
Matching in 3D is more robust than 2D Reprojection (IoU). A small calibration rotation error causes large pixel shifts (ruining 2D IoU), but only small physical shifts (preserving 3D association).

### Fusion Logic: Weighted Average
Step 3 involves merging the data.
*   **Position:** We trust LiDAR (90%) more than Camera (10%).
    `P_final = 0.1 * P_cam + 0.9 * P_lidar`
*   **Color:** We trust Camera (100%). LiDAR is colorblind.
    `Color_final = Color_cam`

---

## 4. Node 4: The Hardware Abstraction Layer

**Role:** Translating "Camera-Speak" to "Car-Speak".
**Source:** `transformer_node.py`

### The Problem
Node 3 outputs data in the **Optical Frame**:
*   `Z` = Forward
*   `X` = Right
*   `Y` = Down

SLAM and Path Planning expect **Vehicle Frame** (REP-103):
*   `X` = Forward
*   `Y` = Left
*   `Z` = Up

### The Solution: Matrix Transformation
Node 4 applies a rotation matrix to swap the axes:

```text
[ X_car ]   [  0  0  1 ]   [ X_opt ]
[ Y_car ] = [ -1  0  0 ] * [ Y_opt ]
[ Z_car ]   [  0 -1  0 ]   [ Z_opt ]
```

It then adds the physical mounting offset definitions from `camera_mounting.yaml`:
*   `x: 1.6` (Camera is 1.6m ahead of rear axle)
*   `z: 0.8` (Camera is 0.8m off the ground)

### Design Rationale
By separating this logic into **Node 4**, we decouple Perception from the Vehicle.
*   **Node 3** is a generic "Cone Detector".
*   **Node 4** is the "Car Adapter".
If we move the camera, we only change Node 4's config. Node 3 remains untouched.

---

## Future Roadmap

With Phase 2 complete, the system outputs reliable, vehicle-centric cone positions using Mock Data. 

**Phase 3 (Simulation Integration)** will focus on replacing this "Mock Data" with high-fidelity sensor streams **received from IPG CarMaker**. This will allow us to validate our clustering and fusion algorithms against realistic physics, sensor noise, and vehicle dynamics before we ever touch a real car.

