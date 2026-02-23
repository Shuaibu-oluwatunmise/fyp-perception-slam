---
title: "Ethics, Risk and Why It Matters for This Project"
date: 2026-02-19 18:00:00 +0000
categories: [Week 5, Project Management]
tags: [risk-assessment, ethics, pde3823, project-management]
pin: false
image:
  path: /assets/img/thumbnails/19.png
  alt: PDE3823 Risk Assessment
  header: false
---

Today wasn't a build day. The module ran a taught session on **ethics and risk assessment** — part of the formal requirements for PDE3823 and, more broadly, for accreditation. Every engineering project at this level needs one, and simulation projects are explicitly not exempt.

## What the Session Covered

The session was straightforward and practical. Ethics in this context isn't abstract philosophy — it's a structured set of questions:

- Who or what could this project impact?
- What assumptions am I making?
- What could go wrong?
- What data am I using and why?
- How will I mitigate any issues?

The risk assessment itself follows a simple formula: **Risk = Severity × Likelihood**. Each identified risk gets scored before and after control measures are applied, showing that the risk has been actively reduced rather than just acknowledged.

The template has two halves: the raw risk (likelihood and severity before controls), and the controlled risk (the same scores after your mitigations are in place). The point is to show the delta — that you've actually thought about what to do, not just listed problems.

## Filling It In for This Project

This project sits in an interesting category. It's primarily a simulation project, but it has real downstream implications — the perception pipeline is intended to eventually run on a physical Formula Student AI vehicle. That makes the risk profile a bit broader than a pure software project.

The risks I identified cover four areas:
- **Computing and hardware** — electrical faults, GPU overheating
- **Health and wellbeing** — prolonged screen use and RSI
- **Project dependencies** — CarMaker licence tied to specific hardware
- **Algorithm and simulation risks** — faulty cone detections, misleading simulation outputs, training data bias, data loss

The one with the highest raw risk level is the algorithm output risk. A false detection or missed cone in a real FS-AI deployment could put marshals or participants at risk. The controls for that one are robust — simulation-first validation, conservative confidence thresholds, documented as a prototype with human oversight — but it's the right one to call out as high.

Here's the completed assessment:

![Risk Assessment Screenshot](/assets/img/blog/2026-02-19/riskassessment.png)
_Completed PDE3823 Risk Assessment for the Perception and SLAM project._

📄 [Download the full Risk Assessment (PDF)](/fyp-perception-slam/assets/docs/blog/2026-02-19/PDE3823%20Project%20Risk%20Assessment%20Template.pdf)

## Back to the Lab

That's the risk assessment done and documented. Tomorrow it's back to the simulation work.
