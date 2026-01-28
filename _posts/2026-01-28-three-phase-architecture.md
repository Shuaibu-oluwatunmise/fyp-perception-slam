---
title: "The Strategy: A 3-Phase Approach to Perception"
date: 2026-01-28 09:00:00 +0000
categories: [Planning, Week 2]
tags: [planning, architecture, roadmap, yolo, fusion]
pin: false
---
After identifying the key components yesterday, I realized that trying to build the entire system at once would be a mistake.

So, before I even touch the code, I sat down today to map out a clear strategy. I’m breaking the **Perception** system down into 3 distinct phases. This way, I can test each part in isolation without getting overwhelmed.

**Note:** I am focusing *only* on Perception for now. The SLAM implementation will come later (around Weeks 7-8) once I have reliable cone detections.

Here is the plan I came up with:

## Phase 1: Camera-Only Perception

The first goal is simply to see. I want to establish a robust pipeline using just the camera.

* **Node 1a:** Camera Publisher (getting images into the system).
* **Node 2a:** Cone Detection (YOLOv8).
* **Node 3:** 3D Localization (projecting 2D boxes into 3D space).

**Target Output:** Accurate cone positions (`x, y, z`) relative to the camera.

## Phase 2: Adding LiDAR & Sensor Fusion

Once the camera pipeline is stable, I will introduce the LiDAR. This is where the **Late Fusion** strategy we chose yesterday comes into play.

* **Node 1b:** LiDAR Publisher.
* **Node 2b:** LiDAR Cone Detection (using clustering algorithms).
* **Node 3 Update:** **Sensor Fusion**. Node 3 will be upgraded to fuse inputs from both Camera (Node 2a) and LiDAR (Node 2b). The output will likely remain in the `camera_frame` for consistency.
* **Node 4:** **Frame Transformation**. A new Node 4 will handle the transformation of these fused cone positions from `camera_frame` to the vehicle's `base_link` frame, making them ready for the SLAM system.

**Target Output:** Fused, redundant cone positions. If one sensor fails, the other backs it up.

## Phase 3: Simulation (CarMaker) Integration

Finally, I need to validate this in a controllable environment before we hit the real track.

* **Integration:** Swap out the data sources (Node 1a/1b) for **IPG CarMaker** streams via CAN/ROS bridge.
* **Logic:** The rest of the pipeline (Nodes 2 & 3) shouldn't even know the difference.

---

This framework gives me a clear path forward. I haven't started implementing any of this yet—I learned the hard way that jumping into coding without a roadmap is not wise.

Now that the strategy is set, I can actually start building **Phase 1**.
