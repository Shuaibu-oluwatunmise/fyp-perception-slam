---
title: "Seeing the Math"
date: 2026-03-05 18:00:00 +0000
categories: [Week 7, Phase 4]
tags: [fusion, lidar, yolo, debugging, NMS, TF2, perception]
pin: false
image:
  path: /assets/img/thumbnails/31.png
  alt: OpenCV Debug View
  header: false
---

Today's stress testing started on the Skidpad. Cones arranged in tight concentric circles — a good scenario to push the perception pipeline. And almost immediately, something odd: the colour labels on the detected cones were twitching.

Not wrong — just unstable. A cone that should be blue would flicker yellow for a frame, then blue again. Another would alternate every few frames. At a glance it looks like a minor visual glitch. In practice it means the fusion engine is genuinely confused about what colour cone it's looking at, and whatever decision gets made downstream is unreliable.

![Colour twitching on the Skidpad](/assets/img/blog/2026-03-05/twitching.gif)
_The colour labels on detected cones twitching during a Skidpad run._

## Why It Happens

The Skidpad is a worst-case scenario for this exact problem. The car drives around a circle of cones, which means from the camera's perspective, cones are constantly stacking up behind each other. A blue cone at 5 metres, a yellow cone at 10 metres directly behind it — from the camera's point of view, their bounding boxes overlap.

![Cone overlap geometry](/assets/img/blog/2026-03-05/conestwitching.png)
_Two cones at different depths — from the camera's perspective, their bounding boxes are close enough to overlap._

When YOLO fires a blue box and a yellow box in the same region, the fusion engine receives two overlapping detections for what appears to be one LiDAR cluster. It has to pick one colour. But the scoring is close enough that it alternates — one frame blue wins, next frame yellow wins. That's the twitching.

The fix requires understanding exactly what the fusion engine is doing at the moment of overlap. Which meant I needed to be able to see it.

## Building the Debugger

The fusion pipeline works in 3D, projecting LiDAR points into the camera frame mathematically. The problem is that when everything is abstract coordinates and matrices, it is very hard to tell whether the projection is actually landing where you think it is.

I built a custom OpenCV debug view that renders the math directly onto the camera image in real time:

- **Green dots** — every LiDAR point projected onto the camera plane
- **Red dot** — the estimated centroid of each detected cluster
- **Blue boxes** — YOLO bounding boxes with confidence and depth labels

For this to be useful the projection has to be accurate — a green dot that lands next to a cone instead of on it tells you nothing. Getting that accuracy required two things: integrating TF2 to fetch real-time sensor transforms (rather than hardcoded static offsets) and correcting the axis permutations to match the camera's optical frame. Once those were in, the dots land where they should.

![OpenCV debug view showing projected LiDAR points](/assets/img/blog/2026-03-05/debug.png)
_The fusion debug view. Green dots are projected LiDAR points, red dots are centroids, blue boxes are YOLO detections. The full Skidpad layout is visible._

With the debugger running, the projection was confirmed working — green dots landing inside the YOLO bounding boxes, centroids in the right place. That was the validation needed to move forward with removing the buffer.

## Enabling Agnostic NMS

While reviewing the pipeline, one more defensive change was worth making. Standard NMS suppresses overlapping detections only within the same class — a blue box and a yellow box in the same region don't suppress each other. In cases where YOLO is genuinely uncertain about a cone's colour, it can produce both, and the fusion engine ends up with two colour candidates for one physical cone. The system would likely resolve this correctly most of the time, but it introduces unnecessary flicker.

One line: `agnostic_nms=True` in the YOLO inference call. Class-agnostic suppression means overlapping boxes get resolved by confidence alone, regardless of class. More robust, no colour flicker from model uncertainty.

## Removing the ±40px Buffer

With accurate projection now in place, something else could also go. Back in the [February 27th fusion post](/fyp-perception-slam/posts/the-fusion-architecture-rewriting-node-3-twice-to-get-it-right/), a ±40px spatial buffer was added to the LiDAR-to-YOLO matching step. At the time, the projection wasn't calibrated — dots were landing outside bounding boxes due to sensor offset errors, so the search area was expanded to compensate.

That buffer was also making the twitching worse. A 40-pixel expanded box on a cone at 5m can bleed into the bounding box of a cone at 6m right next to it — creating false matches across adjacent detections. With TF2 giving accurate projection, the buffer is unnecessary. Removed. The matching step now works within exact bounding box boundaries.

The debugger confirmed all three fixes together — one detection per cone, dots landing precisely on target, stable colour labels throughout the Skidpad run.

Tomorrow will hopefully be the final day of stress testing — but I won't jinx it.
