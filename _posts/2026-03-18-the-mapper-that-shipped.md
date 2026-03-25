---
title: "The Mapper That Actually Shipped"
date: 2026-03-18 18:00:00 +0000
categories: [Week 9, SLAM]
tags: [slam, mapping, localisation, ros2, landmark, ekf, carmaker]
image:
  path: /assets/img/thumbnails/37.png
  alt: Mapping Pipeline
  header: false
pin: false
---

Yesterday was all research and understanding all I think I could about SLAM. This one is what actually ended up running.

After spending time reading about EKF-SLAM and going through the maths, I did try to put an implementation together. It runs — sort of. But between the integration complexity of wiring it into a full multi-sensor fusion pipeline and the fact that it is genuinely hard to validate whether a probabilistic filter is behaving correctly without ground-truth comparison tooling, I was not confident enough in it to deploy it as the live mapping back-end. And honestly, at this point we are already beyond the original project scope — this is a proof of concept in simulation, and whatever actually ends up on the real car will use far more robust mapping strategies than anything here. So "sort of works" was good enough reason to park it.

So I built something simpler instead. Something I could actually reason about and test incrementally within the scope of what this project is.

---

## The Core Insight

The research made one thing clear: the SLAM problem has two halves. The **mapping half** — tracking cone positions as the car drives — and the **localisation half** — using those cone positions to correct the vehicle's pose estimate.

The localisation half is the hard part. It is where EKF-SLAM and graph-SLAM earn their complexity. And given that this is already beyond the original project scope, implementing and validating the full localisation loop on top of everything else was not a realistic target.

So I focused on the mapping half — building the best possible cone tracking system from the perception pipeline output — and used `/carmaker/odom` for pose. That required some edits to the CMRosIF bridge to get CarMaker actually publishing it, but that is a separate story. It is a deliberate simplification, not a solved problem. On real hardware, odometry drifts, and the localisation loop becomes essential. But for a simulation proof of concept, it gets the system to a point where the mapping behaviour can actually be observed and evaluated.

---

## The Architecture

The mapping pipeline is two ROS2 nodes running in series:

```
/cones  (Cone3DArray, Fr1A frame)
    │
    ▼
Local Mapper
    ├── /local_map/cones          (car frame, for path planner)
    └── /local_map/cones_global   (odom frame, for global mapper)
                │
                ▼
          Global Mapper
              ├── /global_map/cones    (odom frame, persistent)
              └── /global_map/markers  (RViz visualisation)
```

The Local Mapper tracks cones as they come in and produces a clean, confirmed list in both the car frame and the fixed odom frame. The Global Mapper accumulates those confirmed cones across the entire run, building the persistent map that downstream navigation reads from.

---

## The Local Mapper

### Temporal Odometry Matching

The first problem: cone detection messages and odometry messages do not arrive at the same timestamp. If you just use the most recent odometry when projecting a cone detection into the global frame, you are using the car's *current* position to place a cone that was actually detected a few milliseconds ago. At racing speeds, a few milliseconds of odometry lag introduces a consistent forward offset into the map.

The local mapper maintains a rolling 2-second history of odometry messages. When a cone detection arrives, it searches the history buffer backwards for the odometry reading whose timestamp most closely matches the detection timestamp. The coordinate frame transform used to project that cone into the global frame corresponds to where the car actually *was* when it made that detection — not where it is now.

### Data Association

Incoming cone observations are matched against the set of currently tracked cones using **sorted greedy one-to-one matching** within a 1.0 m radius. All candidate pairs within range are enumerated, sorted by distance, and assigned greedily — closest pair first, each observation and each tracked cone matched at most once. It is the same Hungarian-style greedy approximation used in the fusion node, applied here to the mapping problem.

### Source-Aware Position Updates

This is probably the most important design decision in the whole mapper.

The perception pipeline produces cones with a `source` field: `fused` (LiDAR + camera confirmed) or `lidar` (LiDAR only). These have different positional accuracy — a fused cone's position is anchored to LiDAR range measurement, which is the most reliable thing the pipeline produces.

Each tracked cone stores the most trusted source type it has ever been observed with. Position updates using **exponential moving average** (EMA with α = 0.3) are only applied when the incoming observation's source is at least as trusted as the stored one. Once a cone is confirmed as `fused`, a lower-quality observation will not drift its stored position.


### Landmark Lifecycle

Each tracked cone has a hit count and a miss count. A cone is only published downstream once it has been observed at least 3 times (`is_confirmed = True`). This confirmation threshold suppresses ghost detections — a single spurious YOLO false positive never makes it into the map.

The miss detection is **FOV-aware for confirmed cones**: a confirmed cone is only penalised for absence if it falls within the expected camera field of view (1–15 m ahead, within roughly ±20° horizontally). Cones behind the car, at extreme angles, or very far ahead are not penalised for being absent — they are simply not visible from the current position, which is not the same as them not existing.

Cones are removed from tracking when they accumulate too many unexplained absences, when the car has driven more than 25 m from them (operationally irrelevant), or when they are more than 5 m strictly behind the vehicle.

### Colour by Majority Vote

Colour classification is determined by majority vote across all observations of a tracked cone. Each observation's `class_name` increments a vote counter. The colour with the highest count wins. This suppresses momentary misclassifications — a yellow cone occasionally labelled orange under a certain lighting condition does not flip the map entry if the next 5 observations all say yellow.

---

## The Global Mapper

The global mapper takes confirmed cone positions from the local mapper and accumulates them persistently across the entire run — across multiple laps if necessary. It applies the same source-aware priority rules as the local mapper, but with higher inertia: EMA weights of 0.85 (existing) and 0.15 (incoming), compared to the local mapper's α = 0.3.

This is intentional. The global map should be stable and resistant to transient noise. The local mapper is more responsive, refining positions as the car approaches a cone. By the time a cone reaches the global map it has already been through that smoothing pass — the global mapper just needs to accumulate and stabilise.

---

## What This Gives You

At the end of a run, `/global_map/cones` contains a persistent, deduplicated, source-aware list of every cone the car has ever confirmed — each with a position that was updated only by observations at least as trusted as its best-ever source.

It is not full SLAM. The localisation half — correcting the vehicle pose using re-observed landmarks — is not here. That is the part that requires EKF-SLAM or a pose-graph back-end, and it is the part I was not able to validate to a deployable standard within the project timeline. What this is, is the mapping half done properly: landmark initialisation, data association, position estimation, lifecycle management, and persistent accumulation — all in a system I can actually inspect, test, and reason about.

The EKF-SLAM code exists. It might even be mostly right. But for the purposes of this project, this is what runs.
