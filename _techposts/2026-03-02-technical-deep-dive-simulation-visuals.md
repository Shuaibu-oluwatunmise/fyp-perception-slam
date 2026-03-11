---
title: "Technical Deep Dive: Fixing MovieNX Asset Rendering & Building the RViz 3D Visualizer"
date: 2026-03-02 20:00:00 +0000
categories: [Week 7, Technical Deep Dive]
tags: [ipg-carmaker, movienx, rviz, ros2, rendering, unigine, visualization-msgs]

pin: true
---

This post is the definitive technical reference for two visual infrastructure upgrades deployed at the start of Week 7.

The first: fixing a UNIGINE GUID mismatch in the MovieNX renderer that was causing the racecar body and large orange cones to be completely invisible in the CarMaker simulation.

The second: replacing the basic RViz sphere markers with proper 3D `.obj` meshes of the FSAI racecar and traffic cones, and getting the timestamp synchronisation right so the visualisation doesn't lag while the car is moving.

---

## Part 1: The MovieNX Mesh GUID Mismatch

### Background: How UNIGINE Tracks Assets

MovieNX is built on the UNIGINE game engine. UNIGINE uses a GUID-based asset management system. When an `.obj` or `.gltf` mesh is imported into a MovieNX project, the engine generates `.meta` sidecar files that assign a unique GUID (a 40-character hex hash) to every mesh and material. These GUIDs are stored inside the hidden `.import_cache/` directory.

Scene configuration files — the `.node` files we author manually inside `FS_autonomous/MovieNX/data/` — reference assets using `guid://` URIs rather than file paths.

The problem: when assets are re-imported (after a cache cleanup, OBJ update, or environment transfer to a new machine), UNIGINE assigns **brand new GUIDs** to all the regenerated cache files. Any `.node` file that was authored by hand still references the old, now-nonexistent GUIDs. The engine fails silently — no popup, just error log entries and invisible meshes.

### Symptoms and Errors

On startup, MovieNX was logging:

```
Movie NX #128: [ERROR] [Engine] Mesh::info(): unknown format of "guid://b9491bb2712e07a37589ed19a98d09c828ada0ce" file
Movie NX #128: [ERROR] [Engine] Mesh::info(): unknown format of "guid://ebbe15dad7eef0ddd9007766e2df2b0bc97be802" file
Movie NX #128: [ERROR] [Engine] Mesh::info(): unknown format of "guid://c5b36db7068ea8e5bb497abb872ee8f7748d1413" file
QCoreApplication::postEvent: Unexpected null receiver
```

Visible symptoms:
- Racecar body completely invisible (only the four tyres rendered — `FSAI_Wheel.node` happened to have a valid GUID)
- All large orange start/finish cones invisible

### Investigation

**Step 1 — Find which `.node` files contain the broken GUIDs:**

```bash
grep -r "b9491bb2\|ebbe15dad7\|c5b36db7" /home/mdxfsai/FS_autonomous/MovieNX/data/
```

Result:
```
3D/Vehicles/Components/AFS_RaceCar_2024/FSAI_Body.node  → guid://b9491bb2...
TrafficCones/TrafficCone_Large_Orange.node               → guid://ebbe15dad7...
TrafficCones/TrafficCone_Large_Orange.node               → guid://c5b36db7...
```

**Step 2 — Confirm those GUIDs don't exist in the import cache:**

```bash
grep -rn "b9491bb2\|ebbe15dad7\|c5b36db7" /home/mdxfsai/FS_autonomous/MovieNX/data/.import_cache/
```

No matches. The GUIDs in the `.node` files had no corresponding entries in the cache — they were stale IDs from a previous import run.

**Step 3 — Find the current correct GUIDs:**

```bash
grep -r "<guid>" /home/mdxfsai/FS_autonomous/MovieNX/data/.import_cache/ | grep "mesh.meta"
```

| Cached mesh file | Current GUID |
|---|---|
| `3D/.../FSAI_Body.gltf/...mesh.meta` | `d1bf821b76c21a8a754df181949ed7890b179fa5` |
| `TrafficCones/big_plastic_orange.obj/..._orange_cylinder...mesh.meta` | `cadc5e7b88db23a684890f011f9f76fd7afe893d` |
| `TrafficCones/big_plastic_orange.obj/..._white...mesh.meta` | `8d1b050e387100ebd94bdcd0d70ac0e12eb4396a` |

### Fix 1a — `FSAI_Body.node` (Racecar Body)

```diff
- <mesh_name>guid://b9491bb2712e07a37589ed19a98d09c828ada0ce</mesh_name>
+ <mesh_name>guid://d1bf821b76c21a8a754df181949ed7890b179fa5</mesh_name>
```

After this change: the racecar body appeared. The reason only the tyres had been visible before is that `FSAI_Wheel.node` had a valid GUID by coincidence — it wasn't part of the re-import cycle.

### Fix 1b — `TrafficCone_Large_Orange.node` (Mesh GUIDs)

```diff
- <mesh_name>guid://ebbe15dad7eef0ddd9007766e2df2b0bc97be802</mesh_name>
+ <mesh_name>guid://cadc5e7b88db23a684890f011f9f76fd7afe893d</mesh_name>

- <mesh_name>guid://c5b36db7068ea8e5bb497abb872ee8f7748d1413</mesh_name>
+ <mesh_name>guid://8d1b050e387100ebd94bdcd0d70ac0e12eb4396a</mesh_name>
```

After this change the cones loaded — but new errors appeared.

### Round 2 — Surface Name Mismatch

```
Movie NX #129: [ERROR] [Engine] Object::loadWorld(): can't find
"TrafficCone_large.orange_Cylinder.007_plastic_orange_traffic.002Mat"
surface in "project_mount/TrafficCones/TrafficCone_Large_Orange.node" file
```

UNIGINE requires the `<surface name="X">` attribute in a `.node` file to exactly match the **material name** from the originating `.obj` file — not the geometry group name, and not with any `Mat` suffix.

To find the correct names I compared against `TrafficCone_Small_Blue.node`, which worked correctly:

```xml
<!-- Working reference (Small Blue cone): -->
<surface name="plastic_blue_traffic"      material="31a882bb..."/>
<surface name="plastic_white_traffic.003" material="04e03865..."/>
```

Pattern: `material_name_from_obj`, no suffix, no geometry group name.

For the large orange cone, imported from `big_plastic_orange_traffic.obj`, the correct surface names are `plastic_orange_traffic.002` and `plastic_white_traffic.002`.

Finding the material GUIDs from the import cache:

```bash
grep -r "<guid>" /home/mdxfsai/FS_autonomous/MovieNX/data/.import_cache/TrafficCones/big_plastic_orange_traffic.obj/
```

| Material | GUID |
|---|---|
| `plastic_orange_traffic.002.mat.meta` | `158e7cf02bee62c963286682661cc8faf61c282c` |
| `plastic_white_traffic.002.mat.meta` | `4253a38393327e7c020592c9e7a74445b24e1085` |

### Fix 2 — `TrafficCone_Large_Orange.node` (Surface Names + Material GUIDs)

```diff
- <surface name="TrafficCone_large.orange_Cylinder.007_plastic_orange_traffic.002Mat" material="626063c1..."/>
+ <surface name="plastic_orange_traffic.002" material="158e7cf02bee62c963286682661cc8faf61c282c"/>

- <surface name="TrafficCone_large.orange_Cylinder.007_plastic_white_traffic.002Mat"  material="6232d785..."/>
+ <surface name="plastic_white_traffic.002"  material="4253a38393327e7c020592c9e7a74445b24e1085"/>
```

### Final Result

All errors cleared. All three scenario types benefit because the asset files are shared:

| Scenario | Car Body | Large Orange Cones |
|---|---|---|
| TrackDrive | Visible | Visible |
| SkidPad | Visible | Visible |
| Acceleration | Visible | Visible |



### Prevention Guide

When assets are re-imported (after OBJ/GLTF updates or cache cleanup), hand-authored `.node` files must be updated manually. The process:

1. Find new mesh GUIDs: `grep -r "<guid>" MovieNX/data/.import_cache/ | grep "mesh.meta"`
2. Find stale references in node files: `grep -r "old_guid" MovieNX/data/`
3. Update mesh GUIDs in the `.node` file
4. Verify surface `name=` attributes match the **material name from the OBJ** — not the geometry group name, no `Mat` suffix
5. Find updated material GUIDs: `grep -r "<guid>" MovieNX/data/.import_cache/<subfolder>/`

---

## Part 2: The RViz 3D Mesh Visualizer

### Why We Needed This

Standard RViz markers — spheres for cones, arrows for the car — are adequate for initial debugging. But when you're trying to verify precise spatial transformations and sensor-to-sensor calibration, looking at coloured balls isn't good enough. You need to see exactly where the car body is relative to the LiDAR scan, and whether the cones are landing where the camera sees them.

The `cone_visualizer.py` node replaces all of those abstract shapes with the actual `.obj` meshes from the simulation.

### Mesh File Setup

RViz accepts `.obj`, `.stl`, and `.dae` files as marker resources. We use the same `.obj` files that MovieNX renders, stored in `fsai_perception/meshes/`. The path format must be `file:///` prefixed absolute path:

```python
_PKG = 'file:///home/mdxfsai/fsai_ws/src/perception/fsai_perception/meshes'

CONE_MESH = {
    'blue_cone':         f'{_PKG}/cones/TrafficCone_Small_Blue.obj',
    'yellow_cone':       f'{_PKG}/cones/TrafficCone_Small_Yellow.obj',
    'orange_cone':       f'{_PKG}/cones/TrafficCone_Small_Orange.obj',
    'large_orange_cone': f'{_PKG}/cones/TrafficCone_Large_Orange.obj',
}

CAR_BODY_MESH  = f'{_PKG}/car/AFS_RaceCar_2024.obj'
CAR_WHEEL_MESH = f'{_PKG}/car/AFS_RaceCar_2024_wheel.obj'
```

### The Mesh Marker Helper

Every `Marker` that renders an external mesh file requires two specific fields:

```python
from visualization_msgs.msg import Marker

def _mesh_marker(frame_id, ns, mid, x, y, z, mesh_path, scale=1.0) -> Marker:
    m = Marker()
    m.header.frame_id = frame_id
    m.ns     = ns
    m.id     = mid
    m.type   = Marker.MESH_RESOURCE   # Must be MESH_RESOURCE, not SPHERE or CYLINDER
    m.action = Marker.ADD
    m.pose.position.x = float(x)
    m.pose.position.y = float(y)
    m.pose.position.z = float(z)
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = scale
    m.mesh_resource = mesh_path
    m.mesh_use_embedded_materials = True  # Apply the colours from the .obj, not a solid ROS colour
    return m
```

`mesh_use_embedded_materials = True` is what makes the cones appear in their actual colours. Without it, RViz renders everything in the default white/grey marker colour regardless of what materials the `.obj` defines.

### Cone Visualisation

When a new `/cones` message arrives, the callback clears all previous markers first to prevent ghosting from frames where the car moved significantly:

```python
def on_cones(self, msg: Cone3DArray):
    markers = MarkerArray()

    # Clear all previous cone markers before drawing the new frame
    delete = Marker()
    delete.header = msg.header
    delete.action = Marker.DELETEALL
    markers.markers.append(delete)

    for i, cone in enumerate(msg.cones):
        if cone.class_name == 'unknown_cone':
            continue  # Don't render colourless detections

        m = _mesh_marker(
            frame_id  = msg.header.frame_id,
            ns        = 'cones',
            mid       = i,
            x         = cone.position.x,
            y         = cone.position.y,
            z         = cone.position.z,
            mesh_path = CONE_MESH.get(cone.class_name, CONE_MESH['blue_cone']),
        )

        m.header.stamp = msg.header.stamp  # Critical — covered below
        m.lifetime.sec = 1
        markers.markers.append(m)

    self.pub.publish(markers)
```

### The Racecar Visualiser

The car is published on a fixed 10Hz timer. It's split into five components:

```python
def _publish_car(self):
    now   = self.get_clock().now().to_msg()
    frame = 'Fr1A'   # Vehicle base frame — rear axle centre
    out   = MarkerArray()

    # Body: OBJ is exported with its origin at Fr1A (rear axle centre)
    body = _mesh_marker(frame, 'car_body', 0, 0.0, 0.0, 0.0, CAR_BODY_MESH)
    body.header.stamp = now
    out.markers.append(body)

    # Wheels: physical offsets relative to Fr1A in metres
    _WHEELS = [
        (2.093,  0.371, 0.257),   # Front Left
        (2.093, -0.581, 0.257),   # Front Right
        (0.557,  0.371, 0.257),   # Rear Left
        (0.557, -0.581, 0.257),   # Rear Right
    ]
    for i, (wx, wy, wz) in enumerate(_WHEELS):
        w = _mesh_marker(frame, 'car_wheels', i, wx, wy, wz, CAR_WHEEL_MESH)
        w.header.stamp = now
        out.markers.append(w)

    self.pub_car.publish(out)
```

Because the car meshes are pinned to the `Fr1A` coordinate frame, the 3D model automatically follows the car as the SLAM system moves `Fr1A` around the map.

### The Most Important Part: Timestamp Synchronisation

This is the detail that caused the most confusion during implementation and is easy to get wrong.

The perception pipeline has latency. By the time a camera frame is processed by YOLO, fused with the LiDAR, and the resulting cone list reaches the visualiser, somewhere between 50 and 200ms has passed. During that time, the car has moved.

If you create the RViz marker with `m.header.stamp = now` (the current system time), RViz uses the car's *current* TF position to place the cone. But the sensor saw the cone when the car was further back. The result: cones appear to be floating ahead of where they actually are relative to the car's current position, drifting further off at higher speeds.

The fix is strict timestamp inheritance:

```python
# The marker must inherit the sensor's original capture timestamp — not the current time.
m.header.stamp = msg.header.stamp
```

By doing this, RViz looks up the car's position in the TF history buffer at the exact nanosecond the sensor fired. It places the cone relative to where the car actually was at that moment. At lower speeds this difference is imperceptible. At 10+ m/s it becomes very visible, very quickly.

For the car body itself, this doesn't apply the same way — the car *is* at its current position, so `now` is correct for the vehicle markers. But for all sensor-derived detections, the sensor timestamp must be passed through.


