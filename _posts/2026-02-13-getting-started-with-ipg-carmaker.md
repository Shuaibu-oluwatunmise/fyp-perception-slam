---
title: "Getting Started with IPG CarMaker: The Simulation is Real"
date: 2026-02-13 18:00:00 +0000
categories: [Week 4, Phase 3]
tags: [ipg-carmaker, simulation, setup, hil, license, rsds]
pin: false
image:
  path: /assets/img/thumbnails/17.jpg
  alt: IPG CarMaker First Run
  header: false
---

> **Note:** For the Phase 2 technical architecture, see the [Technical Deep-Dive: Phase 2](/fyp-perception-slam/techposts/phase-2-technical-deep-dive/). Today we move into Phase 3—simulation integration.

Yesterday, I said tomorrow we'd start looking into the actual **IPG CarMaker** simulation software. Today, that's exactly what happened.

## A Week in the Making

This didn't start today. Back on **February 5th**, I put in the license application. IPG CarMaker is professional simulation software—it's not something you can just download and trial. Access requires going through an official channel, and the license itself is tied to your specific machine.

To apply, I had to provide three pieces of hardware-specific information:
- **PC Hostname** — the name of the machine
- **MAC Address** — the hardware identifier of the network adapter
- **Operating System** — the exact platform the license is generated for

The license is generated specifically to those three criteria, so it is tied to that exact machine. It's not impossible to move — IPG does have a license transfer application process — but it's not something you can just copy across.

I submitted the application and waited. The license arrived, and today I finally had time to act on it.

## Downloading It

Access to the CarMaker installer isn't public. I was provided with specific login credentials to IPG's **Customer Area**—a portal that not everyone can get into. From there, I downloaded the installer and, alongside it, a PDF guide on how to get everything set up.

📄 [IPG CarMaker Installation Guide](/fyp-perception-slam/assets/docs/blog/2026-02-13/InstallationGuide.pdf)

## The Setup Process

Installation itself was straightforward enough—run the installer, follow the steps. The interesting part was the license file.

One thing that caught me out: IPG sends the license file via email, and email clients automatically append a `.dat` extension to it. The instructions are clear — **remove that extension entirely**. The file must have no extension at all: no `.dat`, no `.lic`, nothing. Just the bare filename `Licences`, placed in **`C:\IPG\etc\Licences`**. The software is strict about this, so if anyone's following along and it's not working, that's the first thing to check.

Once the license file was placed correctly and everything was pointed at the right directory, I launched CarMaker for the first time following the guide step by step.

## The First Run

I also watched this getting-started tutorial to help orient myself before diving into the interface — [IPG CarMaker Tutorial](https://www.youtube.com/watch?v=SAvlih6oCWk&t=2s). It gave a solid overview of how projects and test runs are structured.

Here's what the first run of CarMaker looked like on my machine:

{% include embed/youtube.html id='DQuBF1zV9jc' %}
_First run of IPG CarMaker: the simulation environment loading up for the first time._

It works. And honestly, seeing it actually run after weeks of waiting and a HIL setup that was "connected although one end is to a piece of paper" — that felt significant.

## Digging Deeper: The Reference Manual

After getting the basic simulation running, I didn't just stop there. I started exploring the software further, digging through the documentation to understand what I was actually working with.

I found the **IPG CarMaker Reference Manual**, which goes into considerably more depth than the installation guide:

📄 [IPG CarMaker Reference Manual](/fyp-perception-slam/assets/docs/blog/2026-02-13/ReferenceManual.pdf)

Reading through this introduced me to concepts I hadn't encountered before in the context of CarMaker. Most notably, **RSDS** — the **Raw Sensor Data Stream** — which appears to be the mechanism through which the simulation outputs sensor data. This is likely going to be the key to getting CarMaker talking to my ROS 2 nodes.

There were a few other IPG-specific terms and utilities that I need to understand before I can proceed with integration. That investigation is going to be the focus of the **next post**.

## What's Next

The simulation is up and running. I can launch a test environment. But right now, it's just a car driving around a track that I'm watching—there's no connection to my ROS 2 perception pipeline yet.

The next step is to understand exactly how CarMaker exposes its sensor data—specifically through **RSDS** and the ROS 2 bridge—so I can start replacing the mock data in my pipeline with real simulation streams.

Tomorrow, we go deeper into how CarMaker works and what we need to do to connect it to the rest of the system.
