---
title: "When Two Cones Become One"
date: 2026-03-03 18:00:00 +0000
categories: [Week 7, Phase 4]
tags: [ros2, lidar, dbscan, k-means, clustering, perception]
pin: false
image:
  path: /assets/img/thumbnails/29.png
  alt: K-Means Cone Recovery
  header: false
---

With the LiDAR filters from yesterday in place, I started running through different track configurations to stress-test the system. In both the TrackDrive and Acceleration scenarios, there are cones placed close together. I wanted to see how the detector handled them.

It didn't.

Instead of two detections, the system produced one — planted right between the two physical cones. The raw LiDAR data was fine. The point clouds clearly showed two distinct objects. The problem was in the clustering step.

## The Problem: DBSCAN Doesn't Know Where One Cone Ends

DBSCAN groups points based on density. Two points belong to the same cluster if they're within `eps` metres of each other. It doesn't know anything about physical objects — it just connects nearby points.

When two cones are placed in close proximity — the large orange start/finish cones are the clearest example, but this happens any time two cones are within DBSCAN's clustering radius of each other — the laser returns from both land close enough that DBSCAN groups every single point into one cluster. The raw LiDAR data is correct, the point clouds clearly show two distinct scans. The clustering step just treats all of those returns as one object.

The result: one centroid, planted right between the two real cones.

![DBSCAN merging two close-proximity cones into one detection](/assets/img/blog/2026-03-03/wrongkmeans.png)
_DBSCAN groups the returns from both cones into a single cluster — the detection lands in the middle, between the two real cones._

## The Fix: K-Means Recovery

Instead of letting an oversized cluster just pass through as one wrong detection, I intercept it.

The logic: if a cluster's width exceeds `0.45m` — too wide to be a single cone — but its height still looks like a cone (`≤ 0.6m`), don't delete it. Try to split it.

K-Means with `K=2` finds the two centres of mass within the cluster and assigns every point to whichever centre it's closer to. The merged blob gets sliced down the middle, producing two separate sub-clusters. Each sub-cluster then goes back through the full volumetric validator independently.

```python
from sklearn.cluster import KMeans

# Cluster is too wide to be one cone — but height is valid
if width > 0.45 and height <= 0.6:
    kmeans = KMeans(n_clusters=2, n_init='auto', random_state=0)
    labels = kmeans.fit_predict(cluster[:, :2])

    c1 = cluster[labels == 0]
    c2 = cluster[labels == 1]

    # Each half goes back through the full validator
    valid1, _ = self._validate([c1], header)
    valid2, _ = self._validate([c2], header)
```

If both halves are valid cones — both get accepted. If the oversized cluster was never two cones (a wall section, or some other geometry that happens to be tall and wide) — K-Means still splits it, but each half fails the volumetric check and both get rejected. The filter holds either way.

![Two correctly separated detections after K-Means recovery](/assets/img/blog/2026-03-03/rightkmeans.png)
_After K-Means recovery: the same two cones now detected as two separate objects, each with its own correct localisation._

## Why K=2

Why not K=3, or dynamic K? Because the volumetric filter already handles everything else. Anything wider than `1.5m` is a wall, not a merged cone cluster, and gets rejected before K-Means is even attempted. The practical upper bound for close-proximity cones is K=2 — two cones that drifted inside DBSCAN's radius. If there were three, the scene geometry would produce a cluster so wide it would have already been discarded.

K=2 also keeps it computationally cheap. No model selection, no silhouette scoring — just a single forced split, followed by re-validation.

![Full track scene with all cones individually detected](/assets/img/blog/2026-03-03/proof.png)
_Cones in close proximity, all detected as separate individual objects — including the cluster in the upper right. No merging, no ghost centroids._

The LiDAR detector now handles close-proximity cones correctly — right count, right position. Tomorrow, I finally test YOLO26. Back in the Week 3 training report, I flagged long-range detection as YOLOv8n's main weakness and said I'd come back to it. Time to see if YOLO26's architecture changes actually make a difference.
