---
layout: post
title: "Technical Deep-Dive: Node 3 - 3D Localizer"
date: 2026-02-06 18:00:00 +0000
categories: [Technical Deep-Dive]
tags: [ros2, 3d-localization, pinhole-model, synchronization, depth-fusion, implementation]
pin: false
math: true
---

## Overview

**Node 3** is the **3D Localizer**—the bridge between 2D perception and 3D navigation.

While Node 2a answers **"What is this object?"**, Node 3 answers **"Where is this object relative to the car?"**

In its current camera-only configuration, Node 3 is a geometric solver that fuses semantic information (2D bounding boxes) with metric information (depth maps) to reconstruct 3D positions.

### Key Objectives
- **Reconstruction:** Convert 2D pixels $(u, v)$ + Depth $(Z)$ into 3D coordinates $(X, Y, Z)$
- **Synchronization:** Ensure image and depth data align perfectly in time
- **Robustness:** Filter noisy depth measurements to prevent "ghost cones"
- **Standardization:** Output data in `fsai_interfaces/Cone3DArray` format

---

## Architecture: Late Fusion

Node 3 follows a **Late Fusion** (decision fusion) architecture [^1]. Node 2a makes detection decisions first in 2D space. Node 3 then takes these high-level decisions (bounding boxes) and fuses them with depth data.

This is more robust than early fusion because failures in one stage don't cascade through the entire pipeline. If the depth sensor fails, we still have valid 2D detections.

---

## Mathematical Foundation (Brief)

Node 3 uses the **Pinhole Camera Model** to invert 2D projections. Given pixel $(u, v)$ and depth $Z$:

$$
X = (u - c_x) \cdot \frac{Z}{f_x}, \quad Y = (v - c_y) \cdot \frac{Z}{f_y}
$$

Where $(c_x, c_y)$ is the principal point and $(f_x, f_y)$ is the focal length in pixels. These intrinsic parameters are loaded from a calibration file.

> **Note:** The current implementation uses **estimated camera intrinsics** (not from actual calibration) and **mock depth data** (estimated from vertical position in the image, assuming a ground plane). Real-world deployment will require proper camera calibration and actual depth sensors (stereo camera or LiDAR).

---

## Code Structure

### Class: `Localizer3D`

The heart of Node 3 is the `Localizer3D` class in `localizer_3d.py`.

#### 1. Initialization

The node loads camera intrinsic parameters from a configuration file:

```python
self.declare_parameter('intrinsics_file', 'camera_intrinsics_video.npz')
# Loads fx, fy, cx, cy from .npz file
```

#### 2. Synchronized Callback Setup

Instead of separate callbacks for detections and depth, Node 3 has **one callback** that only fires when both pieces of data are available:

```python
self.ts = message_filters.ApproximateTimeSynchronizer(
    [self.detections_sub, self.depth_sub],
    queue_size=10,
    slop=0.1  # 100ms tolerance
)
self.ts.registerCallback(self.sync_callback)
```

**Why 100ms slop?**  
Deep learning detectors introduce inference latency (~30ms). The depth map doesn't wait for inference. The synchronizer allows pairing the stored "old" depth map with the "newly arrived" detection message that corresponds to it.

#### 3. Processing Loop

```python
def process_camera_detections(self, detections_msg, depth_map):
    cones_3d = []
    
    for detection in detections_msg.detections:
        # Step 1: Extract 2D centroid
        u = detection.bbox.center.position.x
        v = detection.bbox.center.position.y
        
        # Step 2: Determine sampling point (cone BASE, not center)
        bbox_height = detection.bbox.size_y
        v_sample = v + (bbox_height * 0.25)
        
        # Step 3: Sample depth using median patch
        Z = self.sample_depth(depth_map, u, v_sample)
        if Z is None or Z <= 0: 
            continue  # Invalid depth
        
        # Step 4: Back-project to 3D
        X, Y, Z = self.pixel_to_3d(u, v_sample, Z)
        
        # Step 5: Construct message
        cone = Cone3D()
        cone.position.x = X
        cone.position.y = Y
        cone.position.z = Z
        cone.class_name = detection.id
        cone.confidence = detection.score
        cone.source = "camera"
        
        cones_3d.append(cone)
        
    return cones_3d
```

---

## Depth Sampling Implementation

Depth maps suffer from flying pixels, occlusion shadows, and reflections. Node 3 uses **median patch sampling** for robustness:

```python
def sample_depth(self, depth_map, u, v):
    # Grab 5×5 patch around sampling point
    patch = depth_map[v-2:v+3, u-2:u+3]
    
    # Filter out invalid depths (zeros, infinities)
    valid = patch[patch > 0]
    
    # Return median (robust to outliers)
    return np.median(valid) if len(valid) > 0 else None
```

**Why median instead of mean?**  
Mean is sensitive to outliers. One pixel reading "100m" (infinite depth) would ruin the average. Median ignores outliers—if 40% of pixels are noise, the median still returns the correct depth.

**Why sample at the cone base?**  
Cones are triangular. The geometric center of the bounding box often contains background (trees, sky) if the box is loose. The base (bottom 25% of the box) is guaranteed to be the wide part of the cone.

---

## Topic Architecture

### Subscribed Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/detections` | `vision_msgs/Detection2DArray` | 2D bounding boxes from Node 2a |
| `/camera/depth` | `sensor_msgs/Image` | Depth map from Node 1a |

### Published Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/cones_camera_frame` | `fsai_interfaces/Cone3DArray` | 3D cone positions in camera frame |

### Example Output

When you run `ros2 topic echo /cones_camera_frame`, you see:

```yaml
cones:
  - position: {x: 2.5, y: 0.3, z: 8.1}
    class_name: "blue_cone"
    confidence: 0.87
    source: "camera"
  - position: {x: -1.2, y: 0.4, z: 12.5}
    class_name: "yellow_cone"
    confidence: 0.92
    source: "camera"
```

---

## Troubleshooting & Debugging

### 1. "No 3D Cones Published"

**Symptoms:** `ros2 topic echo /cones_camera_frame` shows nothing.

**Diagnosis:**
- Check if `/detections` and `/camera/depth` are publishing:
  ```bash
  ros2 topic hz /detections
  ros2 topic hz /camera/depth
  ```
- Check timestamp alignment:
  ```bash
  ros2 topic echo /detections/header/stamp --once
  ros2 topic echo /camera/depth/header/stamp --once
  ```
  If they drift by >100ms, the synchronizer will drop them.
- Check intrinsics loading: Look for errors in terminal output about missing `.npz` file.

### 2. "Cones Positioned Underground or in the Sky"

**Cause:** Bad intrinsic parameters ($f_y$ or $c_y$ is wrong).

**Fix:** Recalibrate the camera using checkerboard calibration and update the `.npz` file.

### 3. "Distance Oscillates Wildly"

**Cause:** Depth noise.

**Fix:** 
- Increase `depth_sample_size` from 5×5 to 7×7 or 9×9
- Check if camera lens is dirty
- Verify depth map quality with `ros2 topic echo /camera/depth`

---

## Design Decisions & Rationale

### Why Separate Localization (Node 3) from Detection (Node 2a)?

We could have put `pixel_to_3d` inside Node 2a. Why didn't we?

- **Bandwidth Efficiency:** Node 2a runs on GPU. Node 3 runs on CPU. By passing small 2D bounding boxes, we avoid passing massive 3D point clouds into the GPU context.
- **Modularity:** We can swap Node 3 for a "Stereo Disparity Node" or "LiDAR Projection Node" without touching the YOLO detector.

### Why Custom Messages (`Cone3D`)?

Why not just publish `visualization_msgs/MarkerArray` directly?

- **Semantic Data:** Markers are just "drawings" (red sphere at XYZ). They lose the concept of *what* the object is.
- **Machine Readability:** `Cone3D` preserves class ID, confidence score, and source sensor info. This is critical for the path planner to make decisions (e.g., "Trust LiDAR cones more than camera cones").
- **Visualization Separation:** In professional stacks, you publish data for algorithms (`Cone3D`) and use a separate node to convert that data into drawings for humans (`MarkerArray`).

**Note on RViz:**  
Because `Cone3DArray` is a custom message, **you cannot see it in RViz directly**. To verify the node is working, you must use the terminal (`ros2 topic echo`).

---

## Coordinate Frame Convention

Node 3 outputs positions in the **camera optical frame**:
- **+X** = Right
- **+Y** = Down
- **+Z** = Forward (distance from camera)

This follows the standard ROS camera frame convention. Future nodes (Node 4+) will transform these coordinates to the vehicle frame (`base_link`) using TF2.

---

[^1]: J. Fayyad, M. A. Jaradat, D. Gruyer, and H. Najjaran, "Deep Learning Sensor Fusion for Autonomous Vehicle Perception and Localization: A Review," *Sensors*, vol. 20, no. 15, p. 4220, 2020.
