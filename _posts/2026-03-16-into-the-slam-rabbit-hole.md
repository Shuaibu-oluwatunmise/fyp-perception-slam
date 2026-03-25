---
title: "Into the SLAM Rabbit Hole"
date: 2026-03-16 18:00:00 +0000
categories: [Week 9, SLAM]
tags: [slam, mapping, localisation, ekf, graph-slam, research, formula-student]
math: true
image:
  path: /assets/img/thumbnails/36.png
  alt: SLAM Research
  header: false
pin: false
---

So here is a thing that happened.

The perception pipeline works. YOLO detects the cones, LiDAR clusters them, the Hungarian algorithm fuses the two streams into a clean list of 3D cone positions. `/cones` gets published. The whole thing runs in CarMaker. Done.

But then I started actually thinking about what comes after that. Technically, autonomous navigation can work without a pre-built map — the car can react to cones as it sees them in real time. But the problem with that is reaction time. If the car is only working off live detections with no prior knowledge of what is ahead, it might not have enough time to plan and respond, especially at speed. A map changes that. If the system already knows where the cones are from earlier in the lap, it can plan further ahead, navigate more smoothly, and handle situations where a cone is temporarily out of sensor range. So it is not that a map is strictly required — it is that navigation is significantly more reliable with one.

That is the SLAM problem. And once I started reading into it, I genuinely could not stop. This post is the research side — everything I read, what the competitive teams do, and how the different approaches compare.

---

## The Problem, Stated Plainly

SLAM stands for Simultaneous Localisation and Mapping. The name is accurate: you are doing both things at the same time. There is no pre-existing map of the track — the layout is unknown until the car is on it. So the car has to build that map from scratch, in real time, using only what its sensors tell it.

The way Thrun, Burgard, and Fox frame it in *Probabilistic Robotics* [^13] — the textbook on this — is that the system is trying to maintain a probabilistic belief over two unknowns at the same time: where the robot is, and what the map looks like. Formally:

$$p(\mathbf{x}_{1:t},\, m \mid \mathbf{z}_{1:t},\, \mathbf{u}_{1:t})$$

The robot's full trajectory and the map are estimated jointly, given all sensor observations and control inputs up to the current time. Every sensor observation updates both. Every movement updates both. Neither is ever known with certainty; both are continuously estimated from noisy data and imperfect odometry.

The chicken-and-egg structure is real. Accurate mapping requires knowing where you are. Accurate localisation is easier with a map to reference against. Both must be estimated simultaneously from the same stream of sensor data. The entire field of SLAM is essentially the question of how to resolve this mutual dependency without one estimate corrupting the other.

---

## Why Formula Student Is a Structured Version of This Problem

Before going into the methods, it is worth explaining why Formula Student is actually a relatively clean version of the SLAM problem compared to, say, autonomous driving on a public road.

The track is defined entirely by coloured traffic cones. There are no dynamic obstacles. The environment does not necessarily change between laps. And — crucially — every cone is a discrete, identifiable feature. You are not trying to extract structure from an arbitrary scene. The cones *are* the structure.

This means the problem naturally fits **landmark-based SLAM**. Rather than maintaining a dense map of every spatial cell in the environment, you just track the positions of individual cone landmarks. The map is literally a list of `(x, y)` positions with associated colours. That is it.

The track is also a closed loop. The car will eventually return to where it started and observe cones it already saw at the beginning of the first lap. That event — recognising a previously seen landmark from a different pose — is known as **loop closure**, and it is one of the most powerful correction mechanisms in SLAM. When you recognise you are back at a known location, you can correct accumulated drift in the entire trajectory estimate. On a Formula Student track, this happens naturally every lap.

---

## What Competitive Teams Actually Do

The first thing I did was read through published technical material from competitive Formula Student Driverless teams. There is a surprisingly decent amount of it.

**municHMotorsport (TU Munich)** published a paper called *clara: Cone-based Localization for Autonomous Racing* [^14]. It is the most complete public description of landmark-based cone SLAM I found for this exact domain. Their approach centres on EKF-SLAM with a greedy one-to-one data association strategy, specifically tuned for the timing and observation patterns of a Formula Student track. They have won the driverless class at Formula Student Germany multiple times. If a method works at that level consistently, it is worth taking seriously.

**AMZ Racing (ETH Zürich)** and **KTH Formula Student** both use variants of landmark-based SLAM as their core mapping and localisation back-ends. The pattern across all three teams — and frankly across everything I read about Formula Student AI — is consistent: discrete cones as landmarks, some form of probabilistic filter or graph optimisation over those landmarks, and a data association step to match incoming observations to previously seen cones.

This convergence across teams gave me confidence that the landmark-based direction was correct. When the best teams in the world independently arrive at the same architectural decision, that is a signal.

---

## EKF-SLAM: The Classical Approach

The Extended Kalman Filter applied to the SLAM problem is the classical solution, described in detail in Thrun, Burgard, and Fox's *Probabilistic Robotics* [^13] — widely considered the foundational textbook on this topic. If you want to understand the theory from the ground up, it starts there.

The core idea is to maintain a single joint estimate over *both* the vehicle pose and all landmark positions at once. Thrun et al. [^13] define this as one augmented state vector:

$$\boldsymbol{\mu} = \begin{bmatrix} x & y & \theta & l_{1x} & l_{1y} & l_{2x} & l_{2y} & \cdots & l_{nx} & l_{ny} \end{bmatrix}^\top$$

The car's position and heading, plus the coordinates of every cone ever seen — all in one vector. The covariance matrix captures not just individual uncertainties, but the *correlations* between them — if you are uncertain about your own position, that uncertainty propagates into your uncertainty about where the landmarks are. It is a coupled estimate.

The filter runs two alternating steps per time cycle:

**Prediction** — use the motion model to propagate the state forward. Given control inputs (velocity and yaw rate), you can predict roughly where the vehicle went. The motion model linearises the non-linear motion around the current estimate, and process noise reflects how uncertain we are about the motion model itself. Landmark positions are left unchanged during prediction — they are fixed in the world frame.

**Update** — for each cone observation, compute the *expected* observation (the range and bearing you would expect to measure to that landmark given the current state estimate), then compare it to the *actual* measurement. The difference — the innovation — updates both the vehicle pose and the matched landmark's position through the Kalman gain. The covariance update uses the Joseph form to ensure the matrix stays numerically stable.

The critical step between prediction and update is **data association** — deciding which incoming cone observation corresponds to which previously seen landmark in the state vector. This is the hardest part of the whole problem. A wrong association — matching a new cone to the wrong existing landmark — corrupts both the pose estimate and the landmark position, and the EKF has no way to recover from a consistent stream of bad associations. It just diverges.

There is also a known scaling issue. The covariance matrix grows quadratically with the number of landmarks. On a small Formula Student track with 50–80 cones, this is manageable in real time on capable hardware like the Jetson AGX Orin. On a full endurance course with 200+ cones, the cost starts growing fast.

---

## Graph-SLAM: The Modern Alternative

Graph-based SLAM (also known as pose-graph SLAM) takes a different architectural approach that addresses some of EKF-SLAM's weaknesses. It was introduced in work by Grisetti et al. [^15] and has become the dominant paradigm in modern SLAM research.

Rather than maintaining a running filter estimate, graph-SLAM separates two concerns:

**Front-end**: handles sensor processing, data association, and loop closure detection. Produces a stream of spatial constraints between poses.

**Back-end**: performs global optimisation over all accumulated constraints. Runs when a loop closure is detected, correcting the entire trajectory and map simultaneously.

A good way to think about the difference: EKF-SLAM is like writing in pen. Every estimate is committed immediately — each linearisation step is permanent, and if there is any error it gets baked in and carried forward. Graph-SLAM is more like writing in pencil. It accumulates constraints without fully committing to a global correction, and when a loop closure is detected it goes back, erases, and rewrites the entire trajectory with a cleaner estimate.

The map is a graph where nodes represent robot poses at discrete time steps, and edges represent spatial constraints — from odometry between consecutive poses, or from loop closures between non-consecutive poses that observed the same landmark. When a loop closure is detected, the back-end minimises a non-linear least squares cost over all accumulated constraints at once [^15]:

$$F(\mathbf{x}) = \sum_{ij} \mathbf{e}_{ij}(\mathbf{x}_i, \mathbf{x}_j)^\top \boldsymbol{\Omega}_{ij}\, \mathbf{e}_{ij}(\mathbf{x}_i, \mathbf{x}_j)$$

The error term measures how far apart each pair of poses is from what the constraints expect, weighted by how confident we are in each constraint. Back-end solvers like g2o, GTSAM, and iSAM2 handle this optimisation efficiently using sparse matrix factorisation [^16].

The key advantage over EKF-SLAM: graph-SLAM does not incrementally linearise at every step. It defers and amortises the global correction until a loop closure occurs, at which point a globally accurate optimisation is performed over the entire trajectory. This makes it significantly more robust to accumulated linearisation error, particularly over long trajectories.

The trade-off is implementation complexity. A complete graph-based system requires a robust loop closure detector, a constraint builder, and an integrated sparse optimisation back-end. iSAM2 [^16], for example, uses an incremental Bayesian tree structure that is genuinely complex to implement correctly from scratch. For a general-purpose indoor robot, this is fine — you use a library. For a custom Formula Student cone-based system, you would need to implement the entire front-end yourself anyway, and the back-end integration with a landmark-based representation is non-trivial.

---

## SLAM Toolbox and Cartographer: The Practical ROS2 Options

After going through the theory, I looked at what the practical out-of-the-box options were for a ROS2 stack.

**SLAM Toolbox** [^6] is the default SLAM package in ROS2 Navigation, maintained by Steve Macenski — the same person leading ROS2 Nav2. It uses graph-based SLAM with an occupancy grid map representation. Handles loop closure. Well documented. Genuinely well-integrated with the ROS2 ecosystem. I had already marked it as a candidate back in Week 2 [^8].

**Cartographer** [^7] is Google's SLAM offering. Also graph-based, also capable, but the consensus from the comparison paper [^8] and the ROS community is that it is harder to configure correctly for non-standard sensor setups and is less actively maintained in the ROS2 world compared to SLAM Toolbox.

Here is where my thinking started to shift, though.

Both SLAM Toolbox and Cartographer use **occupancy grids** — they build dense maps of which spatial cells in the environment are occupied or free. That is the right representation for a robot navigating a warehouse, a hospital corridor, or a building interior. Arbitrary obstacles with arbitrary geometry. You need the grid because you do not know the shape of the environment ahead of time.

On a Formula Student track, the obstacles are individual cones. Discrete points. The environment has no walls, no corridors, no arbitrary geometry. An occupancy grid is overkill for what is fundamentally a sparse, structured, landmark-based problem. You would be using a sledgehammer to press a button.

---

## What I Concluded

After reading all of this — the papers, the team reports, the textbook chapters, the ROS2 discussions — the same direction kept coming out.

For Formula Student specifically:
- The correct map representation is a **list of cone landmarks**, not an occupancy grid
- The correct estimation approach is **landmark-based** (EKF-SLAM or graph-SLAM), not grid-map SLAM
- The hardest part of the problem is **data association** — matching incoming cone observations to previously mapped cones correctly
- **Loop closure** is genuinely important, because the track is a loop and you will see the same cones again
- **EKF-SLAM** is the natural first implementation — the maths is well understood, the reference implementation in *Probabilistic Robotics* is clean, and it maps directly onto the landmark structure of the problem. Scaling limits at 200+ cones are a real concern but not a day-one problem.
- **Graph-SLAM** is the more robust long-term direction, but requires a complete front-end + back-end stack that is a significant engineering project on its own

I did have a go at implementing EKF-SLAM. Not sure if it is fully correct, not sure if I am even going to deploy it. But the research made it clear enough what the problem actually is — which is a start.

---

*Citations: refer to the [Literature Review](/fyp-perception-slam/literature-review/) page for full paper details, specifically [6], [7], [8], [13], [14], [15], [16].*

[^6]: S. Macenski and I. Jambrecic, "SLAM Toolbox: SLAM for the Dynamic World," *JOSS*, 2021.
[^7]: S. Y. Kim et al., "Improving Sensor Adaptability in Cartographer SLAM," *Sensors*, 2025.
[^8]: A. Kucuksubasi and A. B. Can, "Comparative Performance Analysis of SLAM Toolbox and Cartographer in ROS 2," *Electronics*, 2025.
[^13]: S. Thrun, W. Burgard, and D. Fox, *Probabilistic Robotics*. MIT Press, 2005.
[^14]: A. Isenko et al. (municHMotorsport), "clara: Cone-based Localization for Autonomous Racing," *arXiv*, 2022.
[^15]: G. Grisetti et al., "A Tutorial on Graph-Based SLAM," *IEEE ITS Magazine*, 2010.
[^16]: M. Kaess et al., "iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree," *IJRR*, 2012.
