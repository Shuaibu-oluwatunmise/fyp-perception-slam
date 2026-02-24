---
title: "The Material Rendering Barrier and the LiDAR Wall"
date: 2026-02-18 18:00:00 +0000
categories: [Week 5, Phase 3]
tags: [ipg-carmaker, blender, yolo, lidar, debugging, simulation]
pin: false
mermaid: true
image:
  path: /assets/img/thumbnails/21.png
  alt: Debugging Simulation Textures
  header: false
---

Yesterday's success with the networking bridge gave us a live data stream, but it quickly revealed a new problem: our "vision" was hallucinating. Today was a deep dive into the guts of CarMaker's rendering system and a hard lesson in the limitations of simulated LiDAR.

{% include embed/youtube.html id='SVa91qraH78' %}
_Video: Full system test with corrected materials—YOLO detections are now accurate and vibrant._

## The Case of the Material Barrier

When I first brought the YOLOv8 detector online, I expected to see orange and blue cones. Instead, I saw a sea of detection boxes all labeled as **Blue Cones**.

Looking at the raw RGB feed, I saw why. The cones weren't orange at all—they were a ghostly, pale white with subtle hints of blue shading. Because the model was trained on vibrant, real-world orange traffic cones, it was grasping at those blue pixels and misclassifying everything.

### The Root Cause: Invisible Materials
IPG CarMaker's Formula Student package was originally designed for Linux systems. On Windows, the simulation was struggling to render the `.obj` material files for the traffic cone objects. They were loading as untextured geometry, essentially "losing their skin."

## The Blender Pivot

I noticed that while the cones were colorless, the car model itself looked perfect—vibrant colors, reflections, and all. I dug into the car's configuration and saw it wasn't using `.obj`; it was using **GLTF/GLB**.

I had an idea that felt like a long shot: **What if I replaced the broken `.obj` models with `.glb` equivalents?**

1.  **Blender to the Rescue:** I opened the original CarMaker `.obj` cone models in Blender 3D.
2.  **Re-Orientation:** I verified the orientation and material paths.
3.  **Export:** I exported them as `.glb` files with the same names.
4.  **Hacking the Templates:** Simply having the files wasn't enough. I found the CarMaker template files in `Data/Traffic/Template/<Cone_Type>`. I hunted down the `Movie.Geometry` line and manually pointed it to my new `.glb` files instead of the old `.obj` ones.

{% include embed/youtube.html id='oZ2NlrcPVSo' %}
_Video: The fix process—converting models in Blender and modifying CarMaker templates._

## The Result: Vision Restored

It worked! I restarted the simulation, and the track finally had color. YOLO immediately snapped into place, correctly distinguishing between orange and blue cones with high confidence, as seen in the video at the top of this post.

## The LiDAR Wall

However, this victory was bittersweet. While the camera and depth data are now perfect, I’ve hit a brick wall with the LiDAR.

No matter how I configure the Sensor Cluster or the bridge script, the LiDAR data in RViz looks like a **straight white line** cutting through space. It doesn't look like a scan of the environment; it looks like a geometric collapse.

I've checked the ROS visualizer, the binary stride in the bridge, and even the internal CarMaker logs. All signs point to a fundamental issue with how the simulated LiDAR data is being packaged or interpreted on the Windows side. 

As of tonight, camera data is 10/10, depth is 10/10, but LiDAR is currently a blocker. I'm in a bit of a pickle—I've fixed the eyes, but the "feeling" for distance through LiDAR is still missing.

Next steps? Re-evaluating the entire Windows-to-ROS pathway. 
