---
title: "The Fusion Architecture: Rewriting Node 3 Twice to Get It Right"
date: 2026-02-27 18:00:00 +0000
categories: [Week 6, Phase 3]
tags: [ipg-carmaker, ros2, sensor-fusion, hungarian-algorithm, lidar, yolo, tf2, calibration]
pin: false
image:
  path: /assets/img/thumbnails/27.png
  alt: Sensor Fusion Pipeline Architecture
  header: false
---
Yesterday, the bridge was live. Camera data flowing. LiDAR clusters detected. YOLO firing on real simulation frames. Both perception sensors producing independent output.

Today was about making them talk to each other — and discovering that the first working version was mathematically wrong.

{% include embed/youtube.html id='etKfc1cUOck' %}
_Full fusion pipeline: coloured cone detections appearing in RViz in real time._

---

## Node 3 Rewrites Needed, Before We Had Even Started

The Phase 2 fusion had been running with mock data since Week 4 — green fused markers in RViz, camera colour combined with LiDAR geometry. But "working with mock data" and "working with CarMaker" are two very different things.

The pre-integration Node 3 used **hardcoded extrinsic matrices** calibrated for the physical car's sensor mounting. Those values were completely wrong for CarMaker's virtual sensor positions. The fusion logic was sound, but it was transforming LiDAR points using incorrect geometry — meaning every fused cone position would be wrong the moment real simulation data came in.

Fixing that turned out to be a two-step journey.

---

## First Rewrite — Activating LiDAR Fusion

### Step 1: Replace Hardcoded Transforms with TF2

The original code used a manually written extrinsic calibration matrix to transform LiDAR points from sensor frame into camera frame. That matrix was calibrated for the physical car — completely wrong values for CarMaker.

CarMaker publishes its own TF tree at runtime with the exact sensor positions from the simulation. Switching to TF2:

```python
# Before — hardcoded physical car calibration:
rotated = self.camera_rotation.apply(point)
transformed = rotated + self.camera_translation

# After — live lookup from CarMaker's TF tree:
tf = self.tf_buffer.lookup_transform('Fr1A', 'Lidar_F', rclpy.time.Time())
offset = [tf.transform.translation.x,
          tf.transform.translation.y,
          tf.transform.translation.z]
cone.position.x = lidar_cone.position.x + offset[0]
cone.position.y = lidar_cone.position.y + offset[1]
```

This is a translation-only operation because `Lidar_F` and `Fr1A` share the same orientation — both are vehicle-aligned, not optical frame. The offset is simply the LiDAR's mounting position: (2.921m, 0.0m, 0.163m) relative to `Fr1A` (CarMaker's rear axle reference frame).

### Step 2: Force Z = 0

After TF2 transformation, cones appeared floating ~0.163m above the ground. The `Lidar_F → Fr1A` transform includes the LiDAR's physical height above the Fr1A reference. Cones are on the ground plane. The fix:

```python
cone.position.z = 0.0   # Ground plane always, regardless of LiDAR mounting height
```

### Step 3: Node 4 Deleted

In the original design, Node 3 published in the camera optical frame (Z=forward, X=right, Y=down). Node 4's entire job was to remap those axes to the vehicle frame (X=forward, Y=left, Z=up) and add the camera mounting offset.

After TF2 integration, Node 3 published directly in `Fr1A` — CarMaker's vehicle reference frame. Node 4 had nothing left to transform. It was deleted immediately.

Output topic changed from `/cones_camera_frame` to `/cones`. The `fsai_frame_transformer` package and all its files were removed from the workspace.

---

## The Maths That Broke the First Version

With the first rewrite done, the pipeline was producing fused detections. But when I looked at the output carefully, the camera-estimated 3D positions were noticeably wrong — especially at range. I worked out why.

With CarMaker's camera at 640×480 and `fx=320` (90° horizontal FOV), the pinhole back-projection formula is:

```
lateral = (u - cx) * depth / fx
```

A 5-pixel error in the YOLO bounding box centre — very plausible, even for a well-trained detector — at a cone 25 metres away:

```
lateral_error = 5 * 25 / 320 = 0.39 metres
```

At 25m, a 5-pixel detection error produces a **0.39m lateral positioning error**. At wider FOV or longer range it gets dramatically worse. This isn't a YOLO quality issue — it's fundamental pinhole geometry. The further away the cone, the more a pixel error amplifies into a metric error.

**Conclusion:** Camera depth should not be used as the primary source of 3D position at range. LiDAR gives accurate XY. The camera gives accurate colour. The right fusion strategy was to use each sensor for what it is actually reliable at.

---

## Second Rewrite — Late Fusion, Image Space Matching

The key design shift in the second rewrite: **stop asking "which 3D camera cone is nearest to which 3D LiDAR cone?" and start asking "does this LiDAR cone's image projection land inside a YOLO bounding box?"**

This changes the association from a 3D comparison (accurate LiDAR position vs inaccurate camera depth position) to a 2D comparison that bypasses camera depth entirely.

### The Projection Pipeline

For each LiDAR cone arriving in `Lidar_F` frame:

**1. Transform to camera frame** using `extrinsics_carmaker.yaml` (the rigid translation from LiDAR to camera sensor):

```python
p_cam = R_ext @ p_lidar + T_ext
```

**2. Project to image pixel** using the pinhole model:

```python
u = fx * (X / Z) + cx   # = 320 * (X / Z) + 320
v = fy * (Y / Z) + cy   # = 320 * (Y / Z) + 240
```

**3. Check if the projected pixel falls inside a YOLO bounding box** (with ±40px margin):

```python
def point_in_bbox(u, v, det, margin=40.0):
    xc = det.bbox.center.position.x
    yc = det.bbox.center.position.y
    w2 = det.bbox.size_x / 2 + margin
    h2 = det.bbox.size_y / 2 + margin
    return abs(u - xc) <= w2 and abs(v - yc) <= h2
```

If yes — that LiDAR cone is a candidate match for that YOLO detection.

![Image-space matching: projecting LiDAR points into the camera and checking against YOLO bounding boxes](/assets/img/blog/2026-02-27/image_space_matching.png)
_LiDAR 3D point projected into image space — if it lands inside a YOLO bounding box, we have a match._

### Why ±40px Margin?

A calibration diagnostic logger was added to measure the systematic offset between where LiDAR cones should project and where YOLO bounding boxes are centred. The measurement consistently showed approximately **0.35m forward offset** error in the extrinsics — derived from CarMaker's published sensor positions, not from a formal checkerboard calibration.

At typical cone distances, 0.35m translates to approximately 30–40 pixels. Without the margin, correctly matched pairs were failing the point-in-bbox test because projections landed just outside the box edge. The 40px margin compensates without requiring formal calibration.

### Greedy → Hungarian Algorithm

The first rewrite used a greedy nearest-centre approach: for each LiDAR cone, find the YOLO detection with the nearest centre in pixel space.

The problem: if two LiDAR cones project close together in the image (which happens at range when cones are nearly co-linear from the camera's perspective), the greedy algorithm could assign both to the same YOLO detection while leaving another valid detection unmatched.

The second rewrite replaced this with `scipy.optimize.linear_sum_assignment` — the **Hungarian algorithm** for optimal bipartite matching:

```python
# Build cost matrix
cost = np.full((n_lidar, n_yolo), LARGE)
for i, uv in enumerate(lidar_uvs):
    u, v = uv
    for j, det in enumerate(yolo_dets):
        if self.point_in_bbox(u, v, det, margin=self.bbox_margin):
            du = u - det.bbox.center.position.x
            dv = v - det.bbox.center.position.y
            cost[i][j] = (du*du + dv*dv) ** 0.5

# Optimal global assignment
row_ind, col_ind = linear_sum_assignment(cost)
```

The Hungarian algorithm solves the assignment as a global optimisation — minimising total cost across all pairs simultaneously, with each LiDAR cone matched to at most one YOLO detection and vice versa. No two LiDAR cones can steal the same colour.

### Depth Subscription: Removed

The `ApproximateTimeSynchronizer` and `/camera/depth` subscription were removed entirely. No more dropped frames when depth and YOLO timestamps failed to align within the 100ms window. The pipeline now runs at the LiDAR rate, fusing asynchronously against the latest available YOLO detections.

### What Gets Published

```python
if i in matched_lidar:
    cone.class_name = det.id           # Colour from YOLO
    cone.confidence = yolo_score       # YOLO confidence ~0.85–1.0
    cone.source = 'fused'
else:
    cone.class_name = 'unknown_cone'   # No colour match
    cone.confidence = 0.9              # High but colourless
    cone.source = 'lidar'
```

---

## Calibration Files

### Camera Intrinsics

Before integration, Node 3 loaded `camera_intrinsics_video.npz` — calibrated against the video mock pipeline at 1920×1080 with `fx=fy=800`. CarMaker's virtual camera runs at 640×480 at 90° horizontal FOV, which gives:

```
fx = image_width / 2 = 640 / 2 = 320
```

Using the video intrinsics on CarMaker images compressed the horizontal projection by 2.5× — LiDAR points were landing in completely wrong image positions. A new `camera_intrinsics_carmaker.npz` with `fx=fy=320, cx=320, cy=240` was needed.

### LiDAR-to-Camera Extrinsics

`extrinsics_carmaker.yaml` encodes the rigid transform from `Lidar_F` to the camera sensor. Derived from CarMaker's TF tree:

```
Lidar_F:  (2.921m, 0.0m, 0.163m) relative to Fr1A (rear axle)
Camera:   (1.532m, 0.0m, 0.816m) relative to Fr1A (rear axle)

Δx = 1.532 - 2.921 = -1.389m (camera is 1.389m behind LiDAR)
Δz = 0.816 - 0.163 =  0.653m (camera is 0.653m above LiDAR)
pitch = -15° (camera downward tilt)
```

---

## Launch System

Before integration, the pipeline was launched node-by-node with separate commands. After integration, a single `perception.launch.py` handles everything conditionally:

```bash
# Full CarMaker fusion:
ros2 launch fsai_perception perception.launch.py source:=carmaker use_lidar:=True

# Camera only (no LiDAR):
ros2 launch fsai_perception perception.launch.py source:=carmaker use_lidar:=False

# Original video mode:
ros2 launch fsai_perception perception.launch.py source:=video use_lidar:=False
```

A static TF publisher for `Fr1A → base_link` (identity transform) was also added, so RViz and any downstream navigation stack that expects `base_link` as the fixed frame still works while CarMaker's TF tree uses `Fr1A` as its root.

---

## Camera-Only Mode: A Note on Future Work

When `use_lidar:=False`, Node 3 runs YOLO but publishes no cones. Getting reliable 3D positions from the camera alone — without LiDAR — is an open problem. Using the depth map directly has accuracy limits at range, as shown by the pinhole geometry analysis above.

Getting camera-only mode to a point where it produces positions reliable enough for a path planner is a future improvement. What the right approach looks like — whether that involves the depth map in some capacity, geometric assumptions, or something else — is still to be determined. It's a refinement, not a blocker.

---

## Final Result

After both rewrites, new calibration files, and the launch system consolidation:

```yaml
# ros2 topic echo /cones
cones:
  - position: {x: 4.21, y: 1.83, z: 0.0}
    class_name: "blue_cone"
    confidence: 0.91
    source: "fused"
  - position: {x: 4.18, y: -1.74, z: 0.0}
    class_name: "yellow_cone"
    confidence: 0.88
    source: "fused"
```

Approximately 8 coloured cone detections per frame at 10+ Hz, varying by track scenario. LiDAR XY positions, YOLO colour, Hungarian-matched, published in the vehicle frame.

The perception pipeline is essentially complete — what remains is refinement and clean up. Path planning and vehicle control are downstream components outside this project's scope.

The next focus is **optimisation and testing** — seeing the limitations of the current system, evaluating how it performs at bends and corners, refining the detection parameters, and getting the 3D LiDAR clustering fully operational before introducing SLAM. That's where the project goes from here.
