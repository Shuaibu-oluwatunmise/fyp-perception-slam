---
title: "Node 1a: Camera Publisher - Complete Technical Deep-Dive"
date: 2026-01-31 10:00:00 +0000
categories: [Technical Deep-Dive]
tags: [node-1a, camera-publisher, ros2, opencv, architecture]
pin: false
description: "Comprehensive technical documentation for Node 1a, the camera publisher node that serves as the data acquisition layer for the perception pipeline."
---

## Overview

**Node 1a** is the **Camera Publisher** node in the Formula Student AI perception pipeline. It serves as the **data acquisition layer** for camera-based perception (Phase 1).

### Purpose
- Publish RGB camera images to the perception pipeline
- Provide synchronized depth data (mock or real)
- Support multiple data sources (video, ZED2, CarMaker) with a unified interface
- Enable testing without physical hardware

### Position in Pipeline
```
┌─────────────────┐
│   Node 1a       │  ← WE ARE HERE
│ Camera Publisher│
└────────┬────────┘
         │ /camera/image_raw (RGB)
         │ /camera/depth (Depth)
         ▼
┌─────────────────┐
│   Node 2a       │
│ Cone Detector   │
└────────┬────────┘
         │ /detections (2D bboxes)
         ▼
┌─────────────────┐
│   Node 3        │
│ 3D Localizer    │
└─────────────────┘
```

---

## Architecture & Design Philosophy

### Multi-Source Architecture

Node 1a implements a **source-agnostic design** that supports three data sources:

| Source | Status | Use Case |
|--------|--------|----------|
| **Video (Mock)** | Active | Testing without hardware/simulation |
| **ZED2 Camera** | Ready | Real hardware deployment |
| **CarMaker** | Ready | Simulation integration (Phase 3) |

**Key Design Principle:** Downstream nodes (Node 2a, Node 3) are **completely unaware** of the data source. They always subscribe to:
- `/camera/image_raw`
- `/camera/depth`

This abstraction allows seamless switching between sources **without changing any detection or localization code**.

### Why This Matters

```python
# Downstream nodes don't care about the source
# They just subscribe to standard topics

# Works with video:
ros2 launch fsai_sensors camera.launch.py source:=video

# Works with ZED2:
ros2 launch fsai_sensors camera.launch.py source:=zed2

# Works with CarMaker:
ros2 launch fsai_sensors camera.launch.py source:=carmaker

# Detection node stays the same!
ros2 launch fsai_cone_detector camera_detector.launch.py
```

---

## Programming Language & Libraries

### Language: Python 3

**Why Python?**
- Rapid prototyping and development
- Excellent ROS2 support via `rclpy`
- Rich ecosystem for computer vision (OpenCV, NumPy)
- Sufficient performance for 30 Hz publishing
- Easy integration with existing tools

### Core Dependencies

**1. rclpy** - ROS 2 Python Client Library
```python
import rclpy
from rclpy.node import Node
```
**Purpose:** Node lifecycle management, timer callbacks, publishers

**2. sensor_msgs** - ROS 2 Message Definitions
```python
from sensor_msgs.msg import Image
```
**Purpose:** Standard ROS 2 message definitions for sensor data

**3. cv_bridge** - OpenCV ↔ ROS Bridge
```python
from cv_bridge import CvBridge
```
**Purpose:** Convert between OpenCV images (NumPy arrays) and ROS messages

**4. OpenCV (cv2)** - Computer Vision Library
```python
import cv2
```
**Purpose:** Video file handling and image processing
- `cv2.VideoCapture()` - Read video files
- `cv2.CAP_PROP_*` - Video properties (FPS, resolution, frame count)

**5. NumPy** - Numerical Computing
```python
import numpy as np
```
**Purpose:** Efficient array operations for depth generation

---

## Video Handling

The node uses OpenCV's `VideoCapture` to read video files frame-by-frame:

```python
self.cap = cv2.VideoCapture(video_path)
fps = self.cap.get(cv2.CAP_PROP_FPS)
total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
```

### Looping Logic
```python
ret, frame = self.cap.read()
if not ret:
    if self.loop:
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Restart
        ret, frame = self.cap.read()
    else:
        self.timer.cancel()  # Stop publishing
```

**Why Looping?**
- Continuous testing without manual restarts
- Simulates live camera feed
- Useful for demonstrations and debugging

---

## Mock Depth Strategy

Since video files don't contain depth information, Node 1a generates mock depth data to satisfy downstream synchronization requirements.

### Implementation
```python
def _generate_mock_depth(self, timestamp):
    # Create uniform depth map (all pixels = 5.0 meters)
    depth_array = np.full(
        (height, width), 
        self.mock_depth_distance, 
        dtype=np.float32
    )
    
    # Add realistic noise
    noise = np.random.normal(0, 0.05, depth_array.shape)
    depth_array += noise
    
    # Convert to ROS message
    depth_msg = self.bridge.cv2_to_imgmsg(depth_array, encoding='32FC1')
    depth_msg.header.stamp = timestamp  # Synchronized!
    depth_msg.header.frame_id = 'camera_frame'
    
    return depth_msg
```

### Why Mock Depth?
- Enables testing Node 3 (3D localization) immediately
- Validates pipeline architecture before hardware arrives
- Easy to replace with real depth later

### Critical: Synchronized Timestamps
```python
timestamp = self.get_clock().now().to_msg()
img_msg.header.stamp = timestamp
depth_msg.header.stamp = timestamp  # Same timestamp!
```

If timestamps don't match, Node 3 will reject the pair and the pipeline stalls.

---

## Launch System

The node is configured via ROS 2 launch parameters:

```bash
ros2 launch fsai_sensors camera.launch.py \
    source:=video \
    video_path:=/path/to/video.mp4 \
    fps:=30 \
    loop:=true \
    mock_depth_distance:=5.0
```

### Parameters
- `source` - Data source (`video`, `zed2`, `carmaker`)
- `video_path` - Path to video file
- `fps` - Publishing rate (overrides video FPS if set)
- `loop` - Auto-restart video when finished
- `mock_depth_distance` - Uniform depth value in meters

---

## Design Decisions

### 1. Why Unified Topic Names?

**Decision:** Use `/camera/image_raw` and `/camera/depth` for all sources

**Rationale:**
- Downstream nodes don't need to change
- Easy to swap data sources (video → ZED2 → CarMaker)
- Follows ROS 2 conventions
- Simplifies launch files

**Alternative (rejected):**
```
/video/image_raw      # Video source
/zed2/image_raw       # ZED2 source
/carmaker/image_raw   # CarMaker source
```
Requires changing detection node for each source

### 2. Why Timer-Based Publishing?

**Decision:** Use ROS 2 timer for periodic frame publishing

**Rationale:**
- Consistent publishing rate (important for downstream nodes)
- Decouples video FPS from publishing rate
- Allows FPS override for testing
- Standard ROS 2 pattern

**Alternative (rejected):**
```python
while True:
    frame = cap.read()
    publish(frame)
    time.sleep(1/fps)
```
Blocks ROS 2 event loop and has poor integration with ROS 2 lifecycle

---

## Usage

### Basic Usage
```bash
# Launch camera publisher
source ~/fsai_ws/install/setup.bash
ros2 launch fsai_sensors camera.launch.py

# Verify topics
ros2 topic list | grep camera
# Output:
# /camera/image_raw
# /camera/depth

# Check publishing rate
ros2 topic hz /camera/image_raw
# Output: average rate: 30.123

# View images
ros2 run rqt_image_view rqt_image_view
# Select /camera/image_raw from dropdown
```

### Integration with Detection Pipeline
```bash
# Terminal 1: Camera
ros2 launch fsai_sensors camera.launch.py

# Terminal 2: Detector
ros2 launch fsai_cone_detector camera_detector.launch.py

# Terminal 3: Localizer
ros2 launch fsai_localizer_3d localizer_3d.launch.py

# Terminal 4: View output
ros2 topic echo /cones_camera_frame
```

---

## Summary

**Node 1a (Camera Publisher)** is a well-designed, flexible sensor acquisition node that:

- **Publishes** RGB images and depth data to standard ROS 2 topics
- **Supports** multiple data sources (video, ZED2, CarMaker)
- **Enables** testing without hardware via mock depth
- **Provides** synchronized timestamps for 3D localization
- **Uses** Python for rapid development and easy integration
- **Follows** ROS 2 best practices and conventions
- **Abstracts** data sources from downstream nodes
- **Configurable** via launch file parameters

### Next Steps
- Phase 1: Camera-only perception (complete)
- Phase 2: Multi-sensor fusion (complete)
- Phase 3: Integrate real ZED2 camera and CarMaker simulation

**The camera publisher is production-ready and future-proof!**
