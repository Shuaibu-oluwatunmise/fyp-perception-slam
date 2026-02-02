---
title: "Anticipated Challenges & Development Setup"
date: 2026-01-29 09:00:00 +0000
categories: [Technical Deep-Dive, Week 2]
tags: [ros2, jetson, hil, simulation, dev-diary]
pin: false
---

I’m itching to write code because that's just i would normally work on my side projects. Honestly, the planning phase is necessary, but I really just want to see the node running and data flow. But before I can do that, I need to talk about the hardware—and a major issue that has come up. I sort of saw this coming a while ago, but I was really hoping it would be sorted by now.

## The Dream Setup: Hardware-in-the-Loop (HIL)

Normally, I would just run the code on a laptop and call it a day. But for this project, I need to validate that my system works on the *actual* embedded hardware that would go into a Formula Student car.

This brings in the talk of **Hardware-in-the-Loop (HIL)** testing.

The idea is simple but powerful: we essentially trick the embedded computer (the "Brain") into thinking it's driving a real car, when in reality, it's talking to a simulation (the "World").

Here is the setup I’ve built in the lab:

![IPG CarMaker HIL Setup](/assets/img/blog/2026-01-29/hil_setup.jpg)
_My HIL Setup: The Jetson Orin AGX (the cube) talking to the Simulation PC (the paper) via the PCAN adapter._

It consists of three main parts:

1.  **The Brain:** An **NVIDIA Jetson AGX Orin** running Ubuntu 22.04 and ROS 2 Humble. This is where my code lives.
2.  **The Messenger:** A **PCAN-USB Adapter** that creates a CAN bus link. This mimics the actual vehicle's communication network.
3.  **The World:** A Windows PC running **IPG CarMaker**, simulating the physics, track, and sensors.

The beauty of this is that the Jetson *has no idea* it's in a simulation. It receives CAN messages that look exactly like real sensor data, and it sends steering commands thinking it's turning a real steering rack.

## The Reality Check: "Computer says No"

So, the hardware is partially ready. The cables are connected although one end is to a piece of paper. 

The problem: **The IPG CarMaker license hasn't arrived.**

I have the hardware, but without the license, the "World" doesn't exist. I can't generate camera or LiDAR data.

But, I need to be doing something, I cannot just excuse unproductivity.

In engineering (and life generally, I guess), you can’t let external dependencies block you if you actually have a goal to achieve. If I wait two weeks for a license, that’s two weeks of development time lost. I need a way to build and test my perception pipeline *now*, without the simulator.

## The Solution: A "Substitute Data" Strategy

This is where the modularity of ROS 2 comes in beautifully.

My perception nodes—the ones that will detect cones and calculate positions—don't actually care where their data comes from. They just subscribe to a topic called `/camera/image_raw` or whatever I choose to call it.

Whether that topic comes from a $20,000 simulator, a real camera, or a pre-recorded video file... the node doesn't know the difference.

So, here is the new plan:
1.  **Source Real Data:** I’m going to use actual video footage from Formula Student competitions.
2.  **Build a Substitute Publisher:** I’ll write a node (Node 1a) that reads this video file and publishes it to the `/camera/image_raw` topic, mimicking the live stream.
3.  **Fake the Depth:** Since video doesn't have depth, I'll create a "fake" depth publisher to satisfy the system's requirements for Phase 1 testing.

This allows me to build the *entire* perception pipeline (YOLO detection, localizations) right now. When the CarMaker license finally lands, all I have to do is unplug the "Substitute Publisher" and plug in the "CarMaker Bridge." The rest of the system won't even blink hopefully.

## Moving Forward

It’s annoying to be delayed on the simulation side, but honestly? This might be a blessing in disguise. Testing on real video footage means I’m dealing with real-world lighting and noise from Day 1, rather than perfect simulation data.

Tomorrow, I start building **Node 1a**. It’s time to get some pixels moving.
