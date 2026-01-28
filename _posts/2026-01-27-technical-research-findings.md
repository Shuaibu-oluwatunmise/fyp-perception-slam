---
title: "Key Research Findings: Defining the Baseline Architecture"
date: 2026-01-27 14:00:00 +0000
categories: [Technical Deep-Dive, Week 2]
tags: [research, architecture, yolo, slam, design-decisions]
pin: false
---
After spending the last few days buried in research papers (and honestly, getting a bit overwhelmed by the number of options), I’ve finally started to narrow down the architecture for the perception system.

The Literature Review phase is technically "done," but really, it was just about answering one big question: **What is the most robust way to detect cones on a Formula Student car given the real-time demands of autonomous racing?**

Here is what I found, and more importantly, the strategic decisions I’ve made to get this project moving.

## The Vision Model: Starting with Stability

The biggest debate I had with myself was about the object detector. Obviously, everyone knows **YOLO** (You Only Look Once) is the go-to for real-time detection, but *which* version?

I actually spent a lot of time reading into **YOLO26** [^5], a paper that claims some pretty incredible performance gains over the standard models. The benchmarks are tempting—really tempting. If the claims hold up, it could be a game-changer for long-range cone detection, and part of me really wanted to just jump straight into it.

**However, I made a strategic decision today:** I’m not going to start with the experimental stuff. Not yet.

For the initial "Phase 1" of this system, I’m going to use **YOLOv8**. Why? Because it’s the industry standard right now. It has massive community support, pre-trained weights, and most importantly, I strictly know it runs well on the **Jetson AGX Orin** (the computer i have been provided for the FSAI competition). I need to establish a solid baseline first. If I spend the next three weeks fighting with the compilation of a brand-new model like YOLO26, I’m going to miss my deadlines.

So the plan needs to be:

1. Build the pipeline with **YOLOv8** (the safe bet).
2. Get the perception system reliable.
3. *Then* come back and swap in **YOLO26** to see if we can beat the baseline.

It kind of feels like the scientific method in action—I need a control group before I test the variable.

## Mapping: Why SLAM Toolbox?

The next big piece of the puzzle is SLAM (Simultaneous Localization and Mapping). In the Formula Student world, there seem to be two main camps: **Cartographer** (by Google) and **SLAM Toolbox**.

Cartographer is famous, but scanning the ROS 2 forums, it seems like a pain to configure properly for outdoor environments. **SLAM Toolbox**, on the other hand, is maintained by Steve Macenski (the implementation lead for ROS 2 Navigation) and is natively designed for the ROS 2 ecosystem we are using.

Based on the survey papers I read [^6], Graph-based SLAM (which SLAM Toolbox uses) generally holds up better over large loops than older filter-based methods. Since the FS track is one big loop, loop closure is critical. If the map drifts, the car crashes. Simple as that.

## The Consensus: Late Fusion

Finally, how do we combine the Camera and LiDAR?
Paper [^1] was really insightful here. It compared "Early Fusion" (merging raw pixels and points) vs "Late Fusion" (detecting objects separately and merging the lists).

For a student project, **Late Fusion** seems to be the only sane choice. Debugging a "black box" neural network that takes two complex inputs is a nightmare. With Late Fusion, I can debug the Camera detector in isolation, debug the LiDAR clusterer in isolation, and then just write a simple algorithm to match them up in 3D space.

## Moving Forward

So, the architecture is set:

* **Detector:** YOLOv8 (as the baseline).
* **SLAM:** SLAM Toolbox.
* **Fusion:** Late Fusion strategy.

Now the real work begins. I need to stop reading and start coding. The goal for tomorrow is to sketch out the 3-Phase plan to get us from "empty repo" to "autonomous navigation."

---

***Citations:** Refer to the [Literature Review](/fyp-perception-slam/tabs/literature-review/) page for the full paper details referenced here, specifically [1], [5], and [6].*
