---
title: "The FS_autonomous Package: First Run, First Crash"
date: 2026-02-24 18:00:00 +0000
categories: [Week 6, Phase 3]
tags: [ipg-carmaker, ros2, cmrosif, ubuntu, debugging, gpu, carmaker-fsai]
pin: false
image:
  path: /assets/img/thumbnails/24.png
  alt: Diving into the FS_autonomous Package
  header: false
---

I woke up early today. Genuinely excited. Yesterday we got CarMaker running on Ubuntu properly — drivers working, Xorg sorted, the simulation rendering. Today was supposed to be the day we actually *use* it. Load up the FS_autonomous package, get the ROS topics flowing, and start seeing real sensor data.

It did not go as planned. But more on that later — let's start with what the package actually is, because it is a lot more complex than I expected.

## What Even Is the FS_autonomous Package?

When I downloaded the Formula Student CarMaker package from the IPG customer area, I assumed it was basically a pre-configured CarMaker project with some cone tracks and a vehicle model. It's significantly more than that.

📄 [FSAI Users Guide](/fyp-perception-slam/assets/docs/blog/2026-02-24/FSAI_UsersGuide.pdf)

`FS_autonomous` is a complete simulation environment that bridges **IPG CarMaker** (physics, sensors, road, environment) with a **ROS 2 software stack**. The idea is that CarMaker handles the world, and your ROS 2 nodes handle perception, planning, and control — and the package wires them together.

The way it does this is through something called **CMRosIF** (CarMaker ROS Interface). Here's where it gets interesting: CMRosIF isn't a separate process sitting alongside CarMaker. It's a **shared library** (`libCMNode_ROS2_HelloCM.so`) that gets loaded *inside* the CarMaker process at boot. So a ROS 2 node is literally running inside the simulation engine. That's the bridge.

```
┌─────────────────────────────────────────────┐
│              CarMaker Process               │
│  ┌──────────────┐  ┌──────────────────────┐ │
│  │ Physics/     │  │ CMRosIF Plugin       │ │
│  │ Sensor Sim   │◄─►│ (runs inside CM)     │ │
│  │              │  │ Publishes /pointcloud │ │
│  │              │  │ /odom, /imu/data,    │ │
│  └──────────────┘  │ /Camera, /ObjectList  │ │
│                    └──────────┬───────────┘ │
└───────────────────────────────┼─────────────┘
                                │ ROS 2 DDS
                    ┌───────────▼──────────────┐
                    │  Your Perception Nodes   │
                    └──────────────────────────┘
```

On top of that, the ROS 2 workspace inside the package (`ros/ros2_ws/`) has 7 packages:
- **`hellocm_cmnode`** — the CMRosIF bridge node (the most important one)
- **`carmaker_rsds_client`** — receives GPU-rendered camera images from MovieNX over TCP and republishes them as ROS image topics
- **`hellocm`** — external bridge node that handles the init handshake at simulation start
- **`cmrosutils`** — shared utility library and the job scheduler
- **`hellocm_msgs`** — custom message types for the CarMaker ↔ external node sync
- **`camera_msgs`** — custom message types for the idealized camera sensor
- **`vehiclecontrol_msgs`** — the message type for sending steering/throttle/brake back to CarMaker

The entry point for all of this is a script called `CMStart.sh`. You don't just type `CM` like we did yesterday — you run `source CMStart.sh`, which sources the correct ROS 2 environment and then launches CarMaker with the CMRosIF extension loaded. Everything flows from there.

## First Hurdle: Permission Denied Before We Even Start

Before any of the above could happen, I had to actually build the package. It came as a zip archive, so the first step was to run `./build.sh`.

```
/bin/sh: 1: cannot create .depend: Permission denied
make: [Makefile:86: .depend] Error 2
Assembler messages:
Fatal error: can't create CM_Main.o: Permission denied
```

Files extracted from a zip on Linux are often read-only by default. The fix is straightforward:

```bash
chmod -R u+w ~/FS_autonomous
```

Not the most exciting problem to hit, but definitely annoying when you're excited to get going first thing in the morning.

## Second Hurdle: The Broken Symlink

After fixing the permissions, I ran `./build.sh` again. Different error this time:

```bash
/usr/bin/ld: ./../lib//libcmcppifloader-linux64.so: file format not recognized; treating as linker script
/usr/bin/ld: ./../lib//libcmcppifloader-linux64.so:0: syntax error
collect2: error: ld returned 1 exit status
make: *** [Makefile:67: CarMaker.linux64] Error 1
```

This one is a bit more confusing. `libcmcppifloader-linux64.so` in the project's `lib/` folder is actually a **symbolic link** that's supposed to point to the real versioned library file (`libcmcppifloader-linux64.so.1.0.0`). When the project gets extracted from a zip archive, that symlink breaks. The linker then tries to read the symlink file itself as a shared library, gets confused, and refuses to proceed.

Fix: manually recreate the symlink.

```bash
cd lib
sudo ln -sfn libcmcppifloader-linux64.so.1.0.0 libcmcppifloader-linux64.so
```

After that, the build went through properly. Two hurdles before we've even run the simulation. Great start.

## Getting It Running

After the permission and symlink fixes, the ROS 2 workspace was built and everything was sourced. I ran `./CMStart.sh` from inside the `FS_autonomous` directory.

CarMaker opened. The CMRosIF GUI extension loaded. The `hellocm` node span up in a separate terminal automatically. In the CarMaker GUI, I opened the `FS_autonomous_TrackDrive` test run — a full autocross track with blue and yellow cones — and hit **Start Simulation**.

And for a brief moment, it looked like it was going to work.

```
SIM_START		FS_autonomous_TrackDrive	2026-02-19 05:02:08
  Time	5.014
		CarMaker ROS Node enabled: Mode = 1, SyncMode = 0
  Time	0.002
		Beam table read successfully.
  Time	0.000
SIMULATE		FS_autonomous_TrackDrive
```

The ROS node was enabled. The LiDAR beam table loaded. The simulation started.

If everything had gone to plan, a quick `ros2 topic list` should've shown something like this:

```bash
/carmaker/Camera
/carmaker/ObjectList
/carmaker/VehicleControl
/carmaker/pointcloud
/carmaker/odom
/carmaker/imu/data
/clock
/front_camera_depth/image_raw
/front_camera_depth/camera_info
/front_camera_rgb/image_raw
/front_camera_rgb/camera_info
/hellocm/ext2cm
/tf
/tf_static
```

Real sensor data. Real ROS topics. Everything the perception pipeline needs, flowing natively. That was the dream.

We never got there.

Then:

```
ERROR		GPUSensor A2:127.0.0.1.130: Timeout
SIM_ABORT		FS_autonomous_TrackDrive	0.176s	2.59174e-07m
```

0.176 seconds. Done.

## The GPUSensor Timeout

This is where the day got frustrating.

The error is specific: `GPUSensor A2:127.0.0.1.130: Timeout` — the `.130` suffix corresponds to RSDS port 2212, which is the Depth Camera's SensorCluster. So it's always the depth camera that times out, every single run.

To understand why this happens, you need to know how the camera pipeline works. CarMaker has a concept called **SensorClusters** — groupings of GPU sensors that share a MovieNX renderer instance and a TCP port. The project was originally set up with 3 clusters:
- Cluster 0 → LiDAR (MovieNX process #128, port 2210)
- Cluster 1 → RGB Camera (MovieNX process #129, port 2211)  
- Cluster 2 → Depth Camera (MovieNX process #130, port 2212)

Three clusters = three separate MovieNX processes, all doing GPU ray tracing *simultaneously*, all on a single RTX A2000. On simulation start, all three fire up at once. The GPU has to compile shaders, load scene geometry, and render the first frame for all three. It gets through cluster 0 and cluster 1 fine. By the time it reaches cluster 2, CarMaker's internal GPU sensor timeout has already expired and it fires the abort.

There was also this in the MovieNX logs, across all three instances:

```
[ERROR] [Engine] Mesh::info(): unknown format of "guid://b9491bb2712e07a37589ed19a98d09c828ada0ce" file
[ERROR] [Engine] Mesh::info(): unknown format of "guid://ebbe15dad7eef0ddd9007766e2df2b0bc97be802" file
[ERROR] [Engine] Mesh::info(): unknown format of "guid://c5b36db7068ea8e5bb497abb872ee8f7748d1413" file
```

These are 3 mesh assets that MovieNX can't resolve — likely leftover GUIDs from assets that were renamed or removed at some point. They don't cause the crash directly, but they add initialization overhead at exactly the worst possible moment. Not sure what's behind them yet — something to dig into.

## The Documented "Fix" That Didn't Fix It

The FSAI Users Guide mentions adding a simulation parameter `SimStart.Anim.Wait = 300` to give MovieNX more time to initialize before the simulation starts. Seemed reasonable — I added it to `Data/Config/SimParameters` and tried again.

Same crash. Same 0.176 seconds. The wait parameter bought no measurable time.

## Where We Are

So at the end of today, I have the package built, running, and immediately crashing. I know *what* is breaking (the depth camera GPU sensor timing out) and I have a rough idea of *why* (3 concurrent ray tracing contexts on one GPU). What I don't know yet is how to fix it without just disabling the depth camera entirely.

The original promise of switching to Linux was native ROS 2 sensor data — real point clouds, real camera feeds, no custom bridges. We're not there yet. Tomorrow is going to be a full debugging session.
