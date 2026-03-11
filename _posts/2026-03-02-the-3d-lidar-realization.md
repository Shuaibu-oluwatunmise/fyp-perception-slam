---
title: "Week 7: The Perception Overhaul"
date: 2026-03-02 18:00:00 +0000
categories: [Week 7, Phase 4]
tags: [ros2, lidar, point-cloud, dbscan, perception, filtering, movienx, rviz]
pin: false
image:
  path: /assets/img/thumbnails/28.png
  alt: 3D Volumetric LiDAR Filtering
  header: false
---

Week 7 is a full overhaul of the perception stack.

Back in Phase 3, to get the architecture connected and stable as quickly as possible, I deliberately reduced the LiDAR detector to a 2D plane scanner — stripping the Z-axis before clustering so noise from the sparse simulated point cloud wouldn't overwhelm the pipeline. It worked, and the system connected. But once you remove the Z-axis from a 3D LiDAR, you lose the one physical property that distinguishes a cone from a section of track wall. The car was generating false detections from the surrounding environment.

Week 7 is about fixing that, and everything that followed from it.

![False cone detections from the 2D LiDAR pipeline](/assets/img/blog/2026-03-02/false detections.png)
_The 2D baseline: the grey markers show false detections clustered from the environment — not cones._

## The 3D Volumetric Filter

The fix starts by putting the Z-axis back.

Instead of projecting everything onto a flat XY plane before clustering, I let DBSCAN run in full 3D. Once it produces a cluster, I wrap a bounding box around the entire cloud in XYZ and check the physical dimensions against what the Formula Student rulebook says a cone should be — a small cone base of 228mm × 228mm and a height of 325mm, with tolerances for sensor noise and partial occlusion.

[Download the Formula Student Rules 2024 (PDF)](/fyp-perception-slam/assets/docs/blog/2026-03-02/FS-Rules_2024_v1.1.pdf)

```python
min_x, max_x = np.min(cluster[:,0]), np.max(cluster[:,0])
min_y, max_y = np.min(cluster[:,1]), np.max(cluster[:,1])
min_z, max_z = np.min(cluster[:,2]), np.max(cluster[:,2])

width  = max(max_x - min_x, max_y - min_y)
height = max_z - min_z

# FS cone: base ≤ 285mm, height ≤ 505mm
if width < 0.03 or width > 0.60:
    continue  # Dust noise or a track wall

if height > 0.6:
    continue  # Too tall to be a cone
```

A track barrier is narrow in one dimension but very long horizontally. A cone is roughly symmetric. A wall segment is tall. By measuring all three axes simultaneously, the algorithm can distinguish between these geometries instead of guessing by lateral position. This is also what re-enabled the height validation check (`use_height: True`) that had been deliberately disabled in Phase 3 — with real 3D clouds, height is meaningful again.

The false positives from the track boundaries vanished.

![Stable cone detections after the 3D volumetric filter](/assets/img/blog/2026-03-02/stable.png)
_After: Clean, stable cone detections from the 3D volumetric filter._

## The Intensity Filter

The volumetric filter cleared out the structural noise. The next step toward a more robust detection system was adding a physics-based intensity check to catch anything that slipped through.

Formula Student cones are wrapped in retro-reflective tape. That tape returns laser light very differently from asphalt, grass, or general debris. By incorporating the `Intensity` channel from the point cloud (the 4th float in each return, alongside `X, Y, Z`), we can filter not just on geometry but on optical physics.

The complication is that intensity drops with distance — the Inverse-Square Law. A static cutoff (e.g. `if intensity < 200, reject`) works at close range but would blind the detector to all cones beyond roughly 12 meters. The threshold needs to scale with the distance of the cluster:

```python
max_intensity = np.max(cluster[:, 3])
dist = np.linalg.norm(centroid[:2])

# Inverse-square decay: cones at distance d should return ~20000/d² intensity
dynamic_min_int = max(20.0, 20000.0 / (dist ** 2.0))

if max_intensity < dynamic_min_int:
    continue  # Geometrically valid but optically wrong — not a reflective cone
```

Data-logged intensity values during testing: a cone at 5m returns ~980 intensity, the same cone at 25m returns ~55. The dynamic threshold follows this curve, keeping detections reliable out past 30 meters.

## Upgrading the Cone3D Message

With the LiDAR detector now producing geometrically proven 3D shapes rather than single centroid guesses, I upgraded the `Cone3D` message to pass the full point cloud downstream instead of just the centre.

The old message structure collapsed an entire 3D object down to one dot:

```
# Old Cone3D.msg
std_msgs/Header header
geometry_msgs/Point position
string class_name
```

The new one carries the raw cluster:

```
# New Cone3D.msg
std_msgs/Header header
geometry_msgs/Point position
geometry_msgs/Point[] points   # <-- The full physical point cloud
string class_name
```

The reason this matters: when Node 3 (the fusion engine) checks whether a LiDAR cone matches a YOLO detection, it projects the points into image space and checks whether they land inside the YOLO bounding box. With a single centroid, one pixel error can cause a miss. With the full cluster, you have dozens of points distributed across the cone's volume — and if *any* of them land inside the box, the match registers. It is dramatically more robust.

## Fixing the Simulation Visuals

While all this LiDAR work was happening, I also dealt with something that had been nagging for a while: the simulation visuals were broken.

In MovieNX, the racecar body was completely invisible (only the four tyres rendered), and the large orange cones were missing entirely. I documented the full asset GUID fix in a separate technical post — but in brief: CarMaker's MovieNX renderer is built on UNIGINE, which tracks all mesh and material files using GUIDs stored in `.meta` sidecar files. When assets are re-imported, UNIGINE assigns new GUIDs, but hand-authored `.node` files still reference the old dead GUIDs. Tracking down the new correct GUIDs in the `.import_cache` and rewriting `FSAI_Body.node` and `TrafficCone_Large_Orange.node` fixed everything.

**[Read the Technical Deep Dive: Fixing the MovieNX Asset GUID Mismatch and Building the RViz 3D Visualizer](/fyp-perception-slam/techposts/technical-deep-dive-simulation-visuals/)**

With the simulation rendering properly, I also built a `cone_visualizer.py` node to replace the basic RViz spheres with actual 3D `.obj` meshes of the FSAI racecar and traffic cones. The key implementation detail there — which I also cover in the deep dive — is timestamp synchronisation. If you assign a cone marker `rclpy.time.Time().now()` instead of the original sensor timestamp, the car moves between the sensor capture and the render, and the cones appear to lag or drift. Inheriting the exact sensor timestamp from the message header fixes it completely.

![MovieNX with all assets loaded correctly](/assets/img/blog/2026-03-02/movienxoptimised.png)
_All assets loading correctly in CarMaker MovieNX — the car body and large orange cones now rendering properly._

![3D car mesh and cone meshes in RViz](/assets/img/blog/2026-03-02/rviz3dmesh.png)
_The `ConeVisualizer` node: the assembled racecar body (body + 4 wheels anchored to the `Fr1A` frame) alongside the 3D cone meshes, all synchronized to sensor timestamps._

## Where This Leaves Us

By the end of Monday, the LiDAR detector operates in full 3D for the first time — volumetric validation, physics-based intensity filtering, the Cone3D message carrying the full raw point cloud, and the simulation rendering correctly with proper 3D meshes in RViz.

Tomorrow: more tuning. With the 3D filters in place, the next step is to stress-test the system, see what else breaks, and iterate.
