---
title: "What Am I Actually Building? The Perception + SLAM System Explained"
date: 2026-01-26 18:00:00 +0000
categories: [Technical Deep-Dive, Week 2]
tags: [fyp, perception, slam, autonomous-racing, architecture]
---
## The Big Picture: What Problem Am I Solving?

Alright, let's get into it. What exactly am I building, and why does it matter?

At the highest level, I'm working on **autonomous navigation** for Formula Student AI racing. But here's the thing—autonomous navigation isn't just one thing. It's a whole system made up of interconnected parts, and if any one part fails, the whole thing falls apart.

I like to think of autonomous systems in three main layers:

1. **Sensors** - The eyes and ears of the system
2. **Control** - The brain that makes decisions
3. **Actuators** - The muscles that execute those decisions

My project focuses on that very first layer: **sensors**, specifically the **perception pipeline**. And honestly? This is probably the most critical part of the entire system.

## Why Perception Matters So Much

Here's the deal: **everything else relies on a really good perception pipeline.**

Think about it this way—if your perception system isn't working properly, then everything downstream is working with bad data. Your control algorithms will be making decisions based on incorrect information. Your actuators will be executing flawed logic. It's like trying to drive a car with a dirty windshield—you might think you're doing fine, but you're actually heading straight for a tree.

Perception is what gives the autonomous system **eyes**. It's how the car sees the world, detects obstacles (in our case, racing cones), understands where it is, and makes informed decisions about where to go next.

If perception fails, the whole system fails. That's why getting this right is so important.

## What is Perception, Really?

In the context of autonomous racing, **perception** is all about understanding the environment around the vehicle. This involves:

- **Detecting objects** - Where are the cones? What color are they? How far away are they?
- **Classifying objects** - Is that a blue cone or a yellow cone? Is it a cone at all, or something else?
- **Localizing objects in 3D space** - Converting 2D camera detections into real-world 3D positions

We're using a combination of **camera-based vision** (using models like YOLO for object detection) and **LiDAR** (for depth and distance measurements). The goal is to build a robust perception pipeline that can accurately detect and localize racing cones in real-time.

## What is SLAM, and How Does It Fit In?

**SLAM** stands for **Simultaneous Localization and Mapping**. It's a technique that allows a robot (or in this case, a racing car) to:

1. **Build a map** of its environment
2. **Localize itself** within that map

In autonomous racing, SLAM helps the car understand the track layout by mapping the positions of cones and simultaneously figuring out where the car is relative to those cones. This is crucial for path planning and navigation.

So while **perception** tells the car "there's a cone 5 meters ahead on the left," **SLAM** helps the car say "okay, I've seen that cone before, and based on where it is, I know I'm currently at position X, Y on the track."

Together, perception and SLAM give the car a complete understanding of its surroundings and its position within them. That's what makes autonomous navigation possible.

## My System Architecture (High-Level)

I'm building this as a **highly modular ROS2-based system**. Without going into too much detail (I'll save that for future posts), the architecture is broken down into distinct nodes, each with a specific job:

- **Sensor nodes** - Publish camera and LiDAR data
- **Detection nodes** - Run object detection algorithms (like YOLO) to find cones
- **Localization nodes** - Convert 2D detections into 3D positions
- **SLAM nodes** - Build the map and track the car's position (completing this is a nice-to-have; the perception pipeline is the main priority)

The beauty of this modular approach is that each node can be developed, tested, and improved independently. If I need to swap out the detection algorithm or upgrade the SLAM implementation, I can do that without rewriting the entire system.

## What I Hope to Learn from This Project

Honestly, this project is as much about learning new skills as it is about building a working system. Here's what I'm hoping to gain:

### Simulation and Data Handling

I want to get really good at working with simulation environments (like IPG CarMaker) and handling data from simulated sensors. This is a skill that's hugely valuable in industry because it saves so much time and money. Instead of testing everything on physical hardware, you can validate your algorithms in simulation first.

### Project Management

Managing a multi-month project with clear milestones, deliverables, and timelines is a skill in itself. I want to get better at planning, tracking progress, and staying organized throughout the development process.

### ROS2 System Design

I want to be confident in designing and implementing ROS2 systems from scratch. Understanding how to structure nodes, manage topics, handle data flow, and integrate different components is a core skill for robotics engineering.

### Sensor Fusion and Perception Pipelines

Learning how to combine data from multiple sensors (camera + LiDAR) and build robust perception systems is something I'm really excited about. This is at the heart of modern autonomous systems.

### Real-World Problem Solving

Beyond the technical skills, I want to get better at debugging, troubleshooting, and finding creative solutions when things don't work as expected (which, let's be real, happens a lot in robotics).

## What Does Success Look Like?

For me, success on this project means a few things:

**I can design and implement ROS2 systems** - I want to walk away from this project feeling confident in my ability to build modular, well-structured ROS2 applications.

**I have hands-on experience with simulation and data handling** - I want to be comfortable working with simulation environments, processing sensor data, and validating algorithms in virtual environments.

**I have a portfolio of skills I can apply to future projects** - I already have some personal project ideas brewing, and I want to be able to put these new skills to work on them.

**I've built something that actually works** - At the end of the day, I want a functional perception and SLAM system that can detect cones, map the track, and localize the vehicle in real-time.

If I can achieve like 3 out of those things, I'll consider this project a success.

## What's Next?

Over the coming weeks, I'll be diving deep into implementation. I'll be documenting the process here—sharing what works, what doesn't, and what I learn along the way.

Tomorrow, I'll begin to set up my development environment. By the end of this week, I'm aiming to have at least a ROS2 node (the camera publisher) up and running.

It's going to be a challenging few months, but I'm genuinely excited to see where this goes.

Thanks for following along!

---
