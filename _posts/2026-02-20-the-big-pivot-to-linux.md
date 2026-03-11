---
title: "The Big Pivot: Moving to Linux for Full System Integration"
date: 2026-02-20 18:00:00 +0000
categories: [Week 5, Phase 3]
tags: [ipg-carmaker, linux, ubuntu, ros2, humble, cmrosif, pivot]
pin: false
image:
  path: /assets/img/thumbnails/22.png
  alt: Moving to Ubuntu 22.04
  header: false
---
After two days of fighting with custom Python bridges and fighting the "LiDAR Wall," I had a realization today that changed the entire direction of the project. Sometimes, the most efficient way forward isn't to build a better bridge, but to move to a better neighborhood *(was feeling a bit poetic writing this line xD)*.

## Discovery in the `doc` Folder

While searching for any clue as to why my simulated LiDAR was appearing as a straight line in RViz, I went back to the source. I spent the afternoon digging through the folders of the Formula Student CarMaker package I had downloaded earlier in the week.

Deep inside the `doc` folder, I found the "Smoking Gun": the **FSAI_UsersGuide.pdf**.

[Download the FSAI User&#39;s Guide](/assets/docs/blog/2026-02-20/FSAI_UsersGuide.pdf)

Reading through it, I realized I should have looked at this much sooner. It contained images of perfect, high-fidelity point cloud data being published to ROS topics—the exact thing I had been struggling to replicate with my custom Windows bridge.

## Why My Bridge Failed

Section 5.1 of the guide, "Lidar Sensor Specifics," explained exactly why I had hit a wall.

It turns out that **Lidar RSI** sensors in CarMaker don't just output a list of XYZ coordinates. They output a **beam table**. To get a real point cloud, you have to loop through every outgoing laser beam, link it to its specific horizontal and vertical angle, and calculate the geometry based on the length or time-of-flight.

My custom Python bridge was receiving raw numbers, but it wasn't performing this complex "beam table" reconstruction.

## The Choice: Hack or Pivot?

I had two choices:

1. Spend days trying to reverse-engineer the beam table math in Python.
2. Switch to the officially supported **CMRosIF** (CarMaker ROS Interface).

The catch? **CMRosIF is built for Linux.** Specifically, Ubuntu 22.04 and ROS2 Humble.

The benefits of moving were too big to ignore. By switching to Linux, I gain:

- **Native Point Cloud Publishing:** The interface handles the beam table math automatically.
- **Closed-Loop Control:** I can use the `/VehicleControl` topic to send steering and throttle commands back to the simulator.
- **Official Support:** I’m now using the same stack that IPG and the Formula Student organizers expect teams to use.

## The Next Chapter

I put in the application for my Linux CarMaker license last Monday, and I'm hoping it arrives early next week. In the meantime, I've already started the transition—setting up my dual-boot environment with Ubuntu 22.04 and installing ROS2 Humble.

The Windows bridge served its purpose—it proved we could move data and it let me validate the YOLO detector. But for a stable, professional-grade perception and SLAM system, the move to Linux is non-negotiable.

Running this on ROS2 Humble—the environment it was actually built for—is clearly the way to go (I hope I have not jinxed myself).
