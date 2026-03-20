---
title: "The Physics of LiDAR: From Spinning Mirrors to a Clean Point Cloud"
date: 2026-03-11 18:00:00 +0000
categories: [Week 8, Technical Deep Dive]
tags: [perception, lidar, point-cloud, dbscan, interactive, math]
pin: false
image:
  path: /assets/img/thumbnails/34.png
  alt: LiDAR Physics
  header: false
---

The camera post explained why we can't trust camera depth for physical placement. This post is about LiDAR — specifically, how it physically captures the world and how we turn a raw spinning laser into a reliably detected traffic cone.

## 1. How LiDAR Sees the World

A LiDAR sensor is a spinning laser ruler. A motor rotates a set of laser emitters through 360°, firing pulses at every angle. Each pulse travels at the speed of light and bounces back when it hits a surface. The sensor records the round-trip time and converts it into a distance:

```
d = c × (t₁ − t₀) / 2
```

The result is a **point cloud** — not an image, but a set of (X, Y, Z) coordinates with one extra value: **reflective intensity**. The retro-reflective tape wrapped around every FSAI traffic cone glows bright in intensity, which turns out to be critical for discrimination at range.

Critically, the point cloud doesn't appear all at once. Points accumulate progressively as the beam sweeps. Hit Play below and watch the scan build up — notice how the cone shapes only emerge once enough beam angles have passed over them.

<iframe src="{{ site.baseurl }}/interactive_widgets/lidar_sweep_widget.html" width="100%" height="480px" scrolling="no" frameborder="0" style="border:none; border-radius:12px; margin: 20px 0; box-shadow: 0 10px 30px rgba(0,0,0,0.5);"></iframe>

The number of **vertical channels** also matters. A single-channel LiDAR fires one horizontal ring and is nearly blind to the vertical profile of a cone. Our sensor stacks 16 channels between −15° and +15° of vertical elevation. More channels means more returns per object — and more returns means DBSCAN has more density to work with.

One thing worth knowing before we get to clustering: CarMaker doesn't output XYZ directly. It outputs a raw **beam table** — a 2D grid of distances, one row per vertical channel, one column per horizontal angle. The XYZ reconstruction (`x = d·cos(v)·cos(h)`, etc.) happens in middleware. On Linux, CMRosIF does this automatically before publishing the point cloud topic. The Windows bridge I was using early in the project didn't — which is why RViz showed a straight line instead of a 3D cloud. That's the bug that forced the Linux pivot.

## 2. From Dots to Cones: DBSCAN Clustering

Once we have a proper (X, Y, Z, I) point cloud, we need to group the raw returns into discrete objects. The algorithm is **DBSCAN** — "Density-Based Spatial Clustering of Applications with Noise". It groups points that are within **ε metres** of each other, but only confirms a cluster if it contains at least **min\_samples** points.

This is where the cone detection pipeline lives or dies. The widget below lets you place your own LiDAR returns by clicking on the canvas and see exactly when DBSCAN fires:

<iframe src="{{ site.baseurl }}/interactive_widgets/lidar_dbscan_widget.html" width="100%" height="500px" scrolling="no" frameborder="0" style="border:none; border-radius:12px; margin: 20px 0; box-shadow: 0 10px 30px rgba(0,0,0,0.5);"></iframe>

Notice: clicking once or twice near a spot doesn't trigger detection. It takes enough density to clear the **min\_samples** threshold. Now drag that slider up to 8 and watch a previously detected cone vanish — you need to keep clicking to earn it back.

The ε slider controls the neighbourhood radius — the dashed circles show you exactly what it means geometrically. Tighten ε too far and individually placed points stop connecting even if they are intended to represent the same object. Open it too wide and two nearby cones merge into a single cluster.

In the final pipeline, we settled on **ε = 0.35 m** (a traffic cone is ~0.23 m wide) and **min\_samples = 2** to account for the low point density CarMaker's 1 Hz scan produces at range. A KMeans(K=2) recovery step handles pathological cases where two cones merge.

Next post: the **Logic of Fusion** — how TF2 coordinates stitch the camera and LiDAR frames together, and how the Hungarian Algorithm matches detections across frames.
