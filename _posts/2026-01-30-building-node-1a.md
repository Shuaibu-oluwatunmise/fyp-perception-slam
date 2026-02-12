---
title: "Building Node 1a: The Video Publisher"
date: 2026-01-30 17:00:00 +0000
categories: [Technical Deep-Dive, Week 2]
tags: [ros2, perception, implementation, node-1a, opencv]
pin: false
image:
  path: /assets/img/thumbnails/6.png
  alt: Building Node 1a
  header: false
---
Yesterday, I outlined the "Substitute Data" strategy. Today, I built it.

The goal was simple: get video footage publishing to `/camera/image_raw` so I could start testing the perception pipeline without waiting for the CarMaker license.

## Results

It works.

{% include embed/youtube.html id='htFxsgbLq3o' %}
_Demo: Node 1a publishing video footage to ROS 2 topics at 30 Hz_


I can see the video playing in `rqt_image_view`. The topics are publishing at 30 Hz. The mock depth is flowing alongside the RGB frames with matching timestamps—I could even visualize the depth map in `rqt_image_view` to confirm it was publishing correctly.

More importantly, building this validated my modular architecture. The fact that I could swap data sources (video now, real camera later) without touching downstream code reassured me that the design would work when real hardware arrives.

The pipeline is unblocked. I can now start building **Node 2a** (the YOLO cone detector) without waiting for hardware or simulation access.

## The Core Implementation

Node 1a is a ROS 2 Python node that reads a video file frame-by-frame and publishes it as if it were a live camera stream. The logic is straightforward—use OpenCV's `VideoCapture` to read frames, convert them to ROS messages with `cv_bridge`, and publish at 30 Hz using a timer callback.

For testing, I'm using footage from the [FSAI Chalmers team&#39;s onboard camera](https://www.youtube.com/watch?v=xi28kF3kCN8&t=46s)—actual Formula Student racing data with real track conditions, lighting, and cone placements.

The beauty of this approach is that downstream nodes (the YOLO detector, the 3D localizer) have no idea they're looking at pre-recorded footage. They just subscribe to `/camera/image_raw` and process whatever shows up.

I added a looping feature so the video automatically restarts when it ends. This is useful for continuous testing—I can leave the node running and it just keeps cycling through the footage without manual intervention.

The launch system is also flexible. I can configure the video path, publishing rate, and mock depth parameters without touching the code. Just pass different arguments to the launch file and the node adapts.

> **For a comprehensive technical breakdown of Node 1a's architecture, implementation details, and design decisions, see the [Technical Deep-Dive: Node 1a Camera Publisher](/fyp-perception-slam/techposts/node-1a-camera-publisher/).**

## The Mock Depth Solution

The camera publisher was working, but there was still a problem: Node 3 (the 3D localizer) expects both RGB *and* depth data to calculate cone positions in 3D space.

Obviously, a standard video file doesn't have depth information. But if I just left the depth topic empty, the synchronization logic would block forever waiting for matching messages.

So I created a "mock depth" publisher that generates a simple depth map—essentially a 2D array of floats where every pixel is set to 5 meters, with a bit of random noise added to make it slightly more realistic.

It's not accurate, but it doesn't need to be. The point is to satisfy the pipeline's timing requirements so I can test the synchronization logic and coordinate transformations. When the real ZED2 camera arrives (or when CarMaker finally works), I'll just swap this out for real depth data.

The critical part was making sure the RGB and depth messages had **synchronized timestamps**. If they don't match, Node 3 will reject the pair, and the whole pipeline stalls.

## What I Learned

1. **Mock data is good enough to keep moving.** Perfect is the enemy of done. The mock depth isn't realistic, but it's sufficient for testing the pipeline architecture.
2. **Abstraction is powerful.** Because the downstream nodes don't care about the data source, I can swap video for a real camera later without changing a single line of detection code.

Tomorrow, I start on **Node 2a**. Time to teach the system what a cone looks like.
