---
title: "The Debug Marathon: Breaking the Rules to Fix the Camera Pipeline"
date: 2026-02-25 18:00:00 +0000
categories: [Week 6, Phase 3]
tags: [ipg-carmaker, ros2, debugging, rsds, nvidia, ubuntu, jetson, carmaker-fsai]
pin: false
image:
  path: /assets/img/thumbnails/25.png
  alt: Fixing the CarMaker RSDS Camera Pipeline
  header: false
---

You already know I can't help myself — spoiler: it worked.

{% include embed/youtube.html id='ifViTbFfPw8' %}
_CarMaker running on Ubuntu, ROS topics flowing live on the Jetson. This took a while._

But the journey to get here? That's a story. Yesterday we identified the problem — 3 GPU sensor clusters fighting over a single RTX A2000, with the depth camera always timing out. Today was the full debug marathon to fix it.

Fair warning: this post is long. It was a long day.

## The Diagnosis — 10 Experiments to Understand the Problem

The first thing I had to do was actually understand what was causing the timeout. Was it the resolution? The framerate? RViz eating GPU memory? I ran 10 systematic experiments to rule things out one by one.

| # | What I tried | Result |
|---|-------------|--------|
| 1 | Baseline — all 3 sensors, default settings | Crash @ 0.1–0.33s every time |
| 2 | Drop resolution from 3480×1080 → 640×480 | Still crashes, same timing |
| 3 | Drop update rate from 50/16 Hz → 5 Hz | Slightly longer before crash, still crashes |
| 4 | Disable RViz and RQT visualization | Still crashes |
| 5 | Force NVIDIA GPU with `__NV_PRIME_RENDER_OFFLOAD=1` | No change |
| 6 | Reduce clipping plane from 4000m → 200m | No change |
| 7 | Cap simulation speed with `SimStart.TimeFactor = 1.0` | No change |
| 8 | Change depth format from `depth16` → `rgb` | No change |
| 9 | **2 sensors only — LiDAR + Depth, no RGB** | **Full 299s run, stable at 3.5× real-time** |
| 10 | **2 sensors only — LiDAR + RGB, no Depth** | **Full run — completely stable** |

Experiments 9 and 10 were the tell. Two sensors: fine. Three sensors: crash. It wasn't resolution. It wasn't framerate. It wasn't visualization overhead. The GPU just could not service 3 concurrent MovieNX ray-tracing processes within CarMaker's internal timeout window. The problem was structural — one GPU, three simultaneous ray-tracing contexts, not enough time.

The obvious fix would be to merge two cameras into one SensorCluster so they share one MovieNX process instead of spawning three. But the FSAI Users Guide had already told me that wasn't possible:

> *"The current implementation of the RSDS node in ROS can only handle one camera per cluster. If a cluster has two or more cameras, only one will properly transmit data over ROS... Please put only one CameraRSI per cluster."*

So that option was explicitly documented as unsupported. I tried anyway. Merged the depth camera into the RGB cluster, dropped from 3 clusters to 2. As the guide predicted, only one camera published data. The other was silently dropped.

I had to understand *why*.

## The Detour That Cost Me an Entire Day

Before I got to the source code, I went down a very bad path.

I had read something about disabling the GPU watchdog timer on Linux — and thought maybe if I removed the timeout constraint at the OS level, the GPU would just have more time to initialize all three contexts. The idea was to edit `/etc/X11/xorg.conf` and add `Option "Interactive" "0"` to disable watchdog behaviour.

I edited the file. Saved it. Rebooted.

Ubuntu didn't come back up. Black screen. No display server. Nothing.

On a hybrid GPU laptop (Intel iGPU + NVIDIA discrete), manually editing xorg.conf is one of those things that looks reasonable on paper but is absolutely catastrophic in practice. The display server lost track of which GPU to use and the whole session died.

I couldn't fix it from recovery mode. I couldn't fix it from a live USB. So I did the one thing I really didn't want to do — I went to find the university's system technician, had him wipe the Ubuntu partition and reinstall from scratch.

Then I spent the next few hours redoing everything from Monday:
- Install Ubuntu 22.04 LTS
- Install ROS 2 Humble
- NVIDIA drivers + `prime-select nvidia`
- Switch to Xorg
- Reinstall CarMaker + license
- Build the FS_autonomous workspace
- Fix the permissions. Fix the symlink.

The only silver lining: I knew exactly what to do and in what order, so I got through it significantly faster. But it was still a painful detour.

**Lesson learned:** never touch `/etc/X11/xorg.conf` on a hybrid GPU laptop. Ever.

## The Real Fix — Reading the Source Code

Back on a working system, I finally sat down and actually read the RSDS client source code.

The RSDS (Remote Sensor Data Streaming) protocol is how CarMaker's MovieNX renderer streams image frames to the ROS nodes over TCP. Each frame it sends has a header that looks like this:

```
*RSDS <Channel> <ImageType> <SimTime> <Width>x<Height> <DataLength>
```

That `Channel` field is the key. When a SensorCluster contains multiple cameras, MovieNX sends each camera's frames using a *different channel number* over the **same TCP connection**. The protocol already natively supports multiple cameras per cluster.

So what was the existing RSDS ROS node actually doing with those channel numbers?

```cpp
// The original loop:
get_data(img_, &Channel);

// Publish only if channel matches the configured camera
if (Channel == rsdscfg_.Channel) {
    image_pub_.publish(img_, ci);
}
// All other channels: silently discarded
```

It was receiving every frame from every camera in the cluster. Parsing the channel number correctly. And then throwing away everything except the one it was configured for.

The guide said merging cameras into one cluster was a known limitation. But the limitation was entirely in the ROS code — not in the protocol. The fix was already 70% there.

## Doing What the Guide Said Was Impossible

The fix required changes at 4 levels:

### 1. `carmaker_rsds_client_node.cpp` — Route frames to two publishers

Instead of silently dropping frames from the second channel, add a secondary publisher and route based on channel number:

```cpp
// Before:
if (Channel == rsdscfg_.Channel) {
    image_pub_.publish(img_, ci);
}

// After:
if (Channel == rsdscfg_.Channel) {
    image_pub_.publish(img_, ci);
} else if (has_secondary_camera_ && Channel == secondary_channel_) {
    image_pub_secondary_.publish(img_, ci_secondary_);
}
```

7 new parameters added for the secondary camera: `secondary_channel`, `secondary_camera_name`, `secondary_camera_frame`, plus calibration info. When `secondary_channel = -1` (the default), behaviour is identical to before — no breaking changes to single-camera setups.

### 2. `carmaker_rsds_client.launch.py` — Expose the new parameters

Added `DeclareLaunchArgument` entries for all 7 secondary camera parameters, all defaulting to safe values so existing launches work unchanged.

### 3. `CMNode_ROS2_HelloCM.cpp` — Launch one node per cluster, not one per camera

The original code launched a separate RSDS client node for **every active camera**. With two cameras on the same cluster, that meant two nodes both trying to connect to the same TCP port — a socket conflict.

Replaced with a two-pass approach: first collect all cameras and build a `cluster → cameras` map, then iterate clusters and launch exactly one RSDS node per cluster, passing secondary camera parameters for any cluster with two cameras.

### 4. `Data/Vehicle/Examples_FS/FS_Autonomous` — Merge the Depth camera into Cluster 1

```diff
- SensorCluster.N = 3
+ SensorCluster.N = 2

- SensorCluster.2.ID = 0
- SensorCluster.2.Type = CameraRSI
- SensorCluster.2.Socket = 2212
  ...

- Sensor.5.Ref.Cluster = 2
- Sensor.5.PosInWindow = 0 0
+ Sensor.5.Ref.Cluster = 1
+ Sensor.5.PosInWindow = 0 1
```

2 MovieNX processes instead of 3. The GPU handles it comfortably within the timeout window, and both cameras still publish to their own separate ROS topics.

## The Three Bugs That Came Out of the Fix

Getting the fix working introduced 3 new bugs, each more interesting than the last.

### Bug 1: Stack Smashing

```
*** stack smashing detected ***: terminated
```

The launch command for the RSDS node was built using `sprintf()` into a fixed 512-byte buffer. With two cameras and their full parameter sets, the resulting string exceeded 512 bytes. `sprintf()` has no bounds checking, so it overwrote the stack. Fix: increase the buffer.

```cpp
// Before:
char sbuf[512];

// After:
char sbuf[2048];
```

### Bug 2: Both Cameras Publishing to the Same Topic

After fixing the stack smash, the simulation ran — but both cameras were publishing to `/front_camera_rgb/image_raw`. The depth camera had no topic of its own.

The secondary publisher was created as a local variable inside `init()`. When `init()` returned, the local node went out of scope and the publisher died with it. Even while alive, it wasn't namespaced correctly.

Fix: promote the secondary node to a class member so it stays alive for the lifetime of the RSDS client, and publish to an explicit absolute topic path:

```cpp
// In class declaration:
rclcpp::Node::SharedPtr nhp_sec_;

// In init():
nhp_sec_ = rclcpp::Node::make_shared(sec_cam_name + "_node");
image_transport::ImageTransport it_sec(nhp_sec_);
image_pub_secondary_ = it_sec.advertiseCamera("/" + sec_cam_name + "/image_raw", 1);
```

Result: two clean, separate topics:
```
/front_camera_rgb/image_raw
/front_camera_depth/image_raw
```

### Bug 3: Running `ros2 topic list` Killed the Stream

This one was the most interesting. The cameras were finally streaming correctly — both topics publishing, everything looking stable. I opened a new terminal to inspect what was actually running and ran `ros2 topic list` to get a full view of the active topics.

The RSDS node died immediately:

```
terminate called after throwing an instance of 'rclcpp::exceptions::RCLError'
  what(): failed to create guard condition: the given context is not valid,
          either rcl_init() was not called or rcl_shutdown() was called.
[ERROR] [bash-2]: process has died [pid 64375, exit code -6]
```

Root cause: the entire RSDS receive loop was running inside `init()` — a blocking `while` loop that never returned. `rclcpp::spin()` in `main()` never got control. When `ros2 topic list` is run from any terminal, ROS 2 DDS broadcasts a multicast discovery packet. Every node needs to handle this event. Proper nodes handle it via `spin()`. The RSDS node couldn't — it was stuck in the blocking loop. The context became invalid and it crashed.

Fix: move the TCP receive loop into a background thread so `rclcpp::spin()` runs normally:

```cpp
// Added member variable:
std::thread rsds_thread_;

// End of init() — launches background thread and returns:
rsds_thread_ = std::thread(&RSDS_Client::rsds_loop, this);

// Destructor — joins cleanly:
~RSDS_Client() {
    rsdscfg_.TerminationRequested = 1;
    if (rsds_thread_.joinable()) rsds_thread_.join();
}
```

Also added `EINTR` retry handling in `recv_hdr()` and `get_data()` so signal interrupts don't kill the receive loop even in edge cases.

After this fix: `ros2 topic list`, RViz subscriptions, `ros2 topic echo` — all work while the camera stream runs without interruption.

## Connecting the Jetson

With the RSDS stack finally stable, getting the Jetson to see the CarMaker topics was surprisingly straightforward.

ROS 2 uses DDS multicast for automatic peer discovery. Two machines on the same network with a matching `ROS_DOMAIN_ID` will automatically discover each other's nodes and topics — no manual configuration needed.

```bash
# On both the laptop and the Jetson:
export ROS_DOMAIN_ID=42
```

That's it. Same subnet, same domain ID. The Jetson (`192.168.8.118`) and the laptop (`192.168.8.247`) were already on the same network. Once the domain IDs matched, running `ros2 topic list` on the Jetson showed every CarMaker topic published from the laptop — `/carmaker/pointcloud`, `/front_camera_rgb/image_raw`, `/front_camera_depth/image_raw`, `/carmaker/ObjectList`, all of it.

After everything today — the experiments, the Ubuntu reinstall, the four-level RSDS fix, the three follow-on bugs — the final step to get data onto the Jetson was a single environment variable.

One line. That's all it took.

The perception pipeline now has native, real-time ROS 2 sensor data from a proper physics simulation — point clouds, RGB camera, depth camera, object lists, all flowing as standard ROS messages to the Jetson. This is what the Linux pivot was always about.

## A Couple of Things Still to Note

Not everything is perfectly clean. Two minor things are worth flagging:

**FastDDS SIGSEGV on shutdown** — at the end of every simulation, a segfault fires in FastDDS during ROS node teardown:
```
ERROR    Caught signal SIGSEGV (11)
/opt/ros/humble/lib/libfastrtps.so.2.6 -- DataReaderHistory::get_first_untaken_info
```
This is a known bug in ROS 2 Humble's FastRTPS implementation related to publisher destruction ordering. It doesn't affect any simulation data or results — it only happens during clean shutdown and can be safely ignored.

**RSDS node shutdown via `pkill`** — because `image_transport::CameraPublisher` isn't compatible with ROS 2 Lifecycle/Managed nodes, there's no clean way to remotely shutdown the RSDS client nodes. They get brought down at simulation end via `pkill -f carmaker_rsds_client_node`. This causes a "process has died" message in the terminal at the end of each run. Also harmless, also documented in the Users Guide.

Neither affects anything during the actual simulation. Just noise on shutdown.

Now we can actually start integrating.

