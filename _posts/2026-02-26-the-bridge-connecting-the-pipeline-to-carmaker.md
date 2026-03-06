---
title: "The Bridge: Connecting the Perception Pipeline to CarMaker"
date: 2026-02-26 18:00:00 +0000
categories: [Week 6, Phase 3]
tags: [ipg-carmaker, ros2, bridge, dbscan, lidar, yolo, integration, ubuntu]
pin: false
image:
  path: /assets/img/thumbnails/26.png
  alt: CarMaker Pipeline Integration
  header: false
---

Yesterday we got the RSDS fix working and carmaker data flowing as native ROS 2 topics. Today was the day I actually plugged the perception pipeline into it.

In theory, this should have been straightforward. I had a working perception pipeline. CarMaker was now publishing native ROS 2 topics. You'd think it would just... work.

It didn't. Not at first.

{% include embed/youtube.html id='OYTJaeKwFro' %}
_YOLO detections on the live camera feed and DBSCAN clusters visible on the LiDAR point cloud in RViz — the bridge is live._

Here's everything that had to change to get there.

## Why a Bridge Node Was Needed

CarMaker's ROS 2 interface publishes on its own topic names, in its own data formats, with its own QoS policies. My perception pipeline was built around completely different conventions. Rather than rewrite every node to understand CarMaker's interface, the cleaner approach was a single **bridge node** (`carmaker_bridge.py`) that sits between CarMaker and the rest of the pipeline — translating topic names, data formats, and policies so that everything downstream is completely unaware that it's talking to a simulator.

The bridge has three jobs.

![Architecture: CarMaker to Jetson pipeline](/assets/img/blog/2026-02-26/architecture.png)
_The full pipeline from CarMaker on Windows through the bridge to the perception nodes on the Jetson._

---

### Job 1: Camera Topic Relay

CarMaker publishes camera images on `/front_camera_rgb/image_raw`. My pipeline expects `/camera/image_raw`. The bridge subscribes to CarMaker's topic and republishes unchanged:

```python
def camera_cb(self, msg: Image):
    self.cam_pub.publish(msg)  # Pure relay — no conversion
```

Simple enough. But before this worked, there was a problem: **no messages were arriving at all**.

Two things were wrong at once.

First, the **QoS mismatch**. CarMaker publishes with `BEST_EFFORT` reliability. My subscriber defaulted to `RELIABLE`. In ROS 2, a `RELIABLE` subscriber and a `BEST_EFFORT` publisher are incompatible — the subscription silently receives nothing. The fix was explicitly setting `best_effort_qos` on the camera subscription to match CarMaker's policy.

Second, **image size**. CarMaker was initially configured to publish at a high resolution. The messages were large, and combined with the QoS issue, nothing was getting through. I had already dropped the camera to 640×480 during the RSDS setup to reduce GPU load — at that resolution the messages were manageable, and once the QoS was fixed, everything started flowing.

---

### Job 2: Depth Conversion — The Investigation

CarMaker's depth camera publishes as `mono16` — a 16-bit unsigned integer image. My pipeline's 3D localizer expects `32FC1` — 32-bit float, in metres. So I had to convert, but I didn't know what unit the raw values were in.

I wrote a quick diagnostic script to sample the raw pixel values:

- Centre pixel: ~314
- Median valid value: ~167

Three hypotheses tested:

| Hypothesis | Divisor | Result | Valid? |
|---|---|---|---|
| Values in millimetres | ÷1000 | 0.31m median | No — too close, camera is above ground |
| Values in centimetres | ÷100 | 3.14m median | Yes — matches geometric prediction |
| Values in decimetres | ÷10 | 31.4m median | No — too far |

I cross-validated the centimetre hypothesis geometrically. The camera is mounted at 0.816m height with a 15° downward tilt. Working out the ground-plane intersection distance from the camera's viewing geometry gives ~3.05m. The raw value of ~314 divided by 100 gives 3.14m — within 1% of the geometric prediction. Centimetres it was.

One more thing: CarMaker encodes pixels with no valid data (sky, objects beyond sensor range) as the maximum uint16 value: **65535**. If left in, these become 655.35m depth values that corrupt every 3D position estimate downstream. They get clamped to zero (no-data) before publishing:

```python
depth_metres = depth_image.astype(np.float32) / 100.0
depth_metres[depth_metres > 50.0] = 0.0   # Clamp sky/sentinel values
out_msg = self.bridge.cv2_to_imgmsg(depth_metres, encoding='32FC1')
out_msg.header = msg.header   # Preserve original timestamp!
```

The header timestamp has to be preserved from the original message. Replacing it with `ros::now()` would break temporal alignment for any downstream synchronisers.

---

### Job 3: PointCloud v1 → PointCloud2

CarMaker publishes LiDAR data as `sensor_msgs/PointCloud` — the old, deprecated v1 format that stores points as a `geometry_msgs/Point32[]` array. My DBSCAN detector expects `sensor_msgs/PointCloud2` — the modern binary blob format.

The bridge packs the points manually using Python's `struct` library into the binary format:

```python
def lidar_cb(self, msg: PointCloud):
    buf = bytearray(12 * len(msg.points))
    for i, pt in enumerate(msg.points):
        struct.pack_into('fff', buf, i * 12, pt.x, pt.y, pt.z)
    pc2_msg.data = bytes(buf)
    pc2_msg.header = msg.header   # Preserve Lidar_F frame_id!
```

The `frame_id` in the header has to be preserved. CarMaker's LiDAR publishes in the `Lidar_F` frame, and TF2 needs this to look up the correct sensor-to-vehicle transform at runtime. If I changed it to a generic `lidar` frame, TF2 lookups would fail.

---

## Node 2a (YOLO): Zero Changes

After all the bridge complexity, the YOLO camera detector was the one part that required absolutely nothing.

The YOLOv8n model trained back in Week 3 on the FSOCO dataset was used unchanged. CarMaker's 3D cone models are geometrically accurate replicas of real cones — same shape, same colours. The model generalised directly to simulation frames with no retraining, no parameter changes, no code modifications.

At integration, it was detecting ~12.5 cones per frame. The pipeline abstraction held: Node 2a just subscribes to `/camera/image_raw` and produces detections. It has no idea where those images come from.

---

## Node 2b (DBSCAN): A Near-Complete Rethink

The LiDAR detector needed significantly more attention. Not because the code was wrong — it was correct for what it was designed for. The problem is that **CarMaker's simulated LiDAR produces a fundamentally different point cloud** from what the code was built around. Node 1b's mock data generated dense, clean clusters of ~75 points per cone. CarMaker's LiDAR produces a much sparser and noisier return profile across the scene.

To get the pipeline connected and running robustly from end-to-end, **I made the deliberate decision to flatten the data and treat it as a 2D sensor for this initial integration.** Try to cluster the sparse Z-variation right now was adding noise and breaking detections. Getting a reliable 3D clustering implementation working on this specific sensor profile is planned as a future optimisation, but for now, 2D gives us the stable baseline we need.

### Why Almost Everything Had to Change

Because the points are flattened to a single plane, a cone cluster and a wall segment look identical in height — essentially zero. This breaks almost every assumption the original detector was built on:

#### Ground Removal: Fixed Threshold → Adaptive 10th-Percentile

**Before:**
```python
mask = pts[:, 2] > 0.05   # Fixed height threshold
```
With the data flattened to 2D, ground returns and cone returns share the same Z — a fixed absolute threshold doesn't separate them reliably.

**After:**
```python
self.last_ground_z = np.percentile(pts[:, 2], 10)
mask = pts[:, 2] > (self.last_ground_z + self.ground_threshold)
```
With a single-plane scanner, the ground Z shifts as the car pitches and rolls. The 10th percentile of all Z values is used as an adaptive ground estimate — the lowest 10% of points are assumed to be ground, and only points significantly above that baseline are passed to the clustering stage.

#### Clustering: 3D → XY Only

**Before:** DBSCAN ran in 3D (XYZ).

**After:**
```python
xy = pts[:, :2]
labels = DBSCAN(eps=self.eps, min_samples=self.min_points).fit_predict(xy)
```
Since all points are projected to the ground plane, including Z in the clustering distance metric adds noise without information. XY-only clustering is cleaner and more numerically stable for this initial baseline.

#### min_points: Significantly Reduced

With the mock data, multiple scan lines hit a cone from different vertical angles, guaranteeing a high number of tightly-packed returns. CarMaker's sparser point cloud doesn't have that guarantee — especially at longer ranges where the return count drops significantly.

The original `min_points=10` was rejecting valid distant cones that had fewer returns. Reducing it significantly ensures those cones are detected too — at the cost of accepting more noise, which the footprint size check downstream handles.

#### Height Validation: Disabled

**Before:**
```python
if height < 0.2 or height > 0.4: continue   # Cone must be 20–40cm tall
```

**After:** `use_height_filter=False` in the CarMaker launch config.

Height validation cannot work when all returns share approximately the same Z value. A cone cluster and a rock at the same distance would appear equally tall — zero.

#### Cluster Width Bounds: Widened

| Parameter | Before | After | Reason |
|---|---|---|---|
| width_min | 0.1m | 0.03m | Close cones produce narrow edge-on clusters |
| width_max | 0.3m | 0.60m | Close-range cones can appear up to 60cm wide at angle |

#### Colour Assignment: Removed

**Before:**
```python
cone_class = 'blue_cone' if centroid[1] > 0 else 'yellow_cone'
```

**After:**
```python
cone_class = 'unknown_cone'
```

The lateral position heuristic (left side = blue, right side = yellow) was a rough approximation that only holds on a straight section with the car perfectly centred. In the fused pipeline, colour comes from YOLO, which actually has colour information. Assigning fake colours from position would create conflicts during fusion.

Confidence scoring was also formalised: `min(1.0, len(cluster) / 20.0)` — proportional to point count. A cluster with 20+ returns gets confidence 1.0; one with 5 returns gets 0.25.

#### Reject Logging (New)

A diagnostic logger was added to understand what the detector was rejecting:

```
[DBSCAN] Rejected 3 clusters this frame:
  d=6.8m pts=123 w=6.17m → width OOB
  d=8.5m pts=208 w=16.96m → width OOB
  d=17.4m pts=8  w=1.64m  → width OOB
```

Wide clusters (w > 0.6m) are walls, barriers, or the car body. This logging was essential for distinguishing those from actual cones during parameter tuning.

---

## Where Things Stand

By end of today, the bridge is live. Camera data is flowing, depth is correctly converted, LiDAR point clouds are arriving. YOLO is detecting cones in real time. DBSCAN is finding clusters.

Both detectors are producing output independently. What's not done yet is the most interesting part: taking those two independent streams and combining them into a single, geometrically accurate, colour-labelled list of cones.

That's tomorrow.
