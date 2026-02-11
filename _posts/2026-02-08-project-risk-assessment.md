---
title: "Project Management: Formal Risk Assessment"
date: 2026-02-08 14:00:00 +0000
categories: [Project Management, Week 3]
tags: [risk-management, planning, safety, methodology]
pin: false
---

As we approach the halfway mark, we need to formalize the **Risk Assessment**.

Since the majority of this project is simulation-based and software-focused, the primary risks aren't physical hazards (like crashing a car), but **logistical and resource-based risks** that could derail the project timeline.

## The Risk Matrix

I have categorized the key project risks by **Likelihood** (1-5) and **Severity** (1-5). The **Risk Score** is the product ($L \times S$).

### High Priority Risks (Score >= 15)

| ID | Risk Description | Likelihood (1-5) | Severity (1-5) | Score | Mitigation Strategy |
|:---|:---|:---:|:---:|:---:|:---|
| R1 | **Simulation License Delay** | 5 | 5 | **25** | The IPG CarMaker license is critical. **Mitigation:** De-risk by building a "Substitute Data Pipeline" (Nodes 1a/1b) that uses recorded data instead of live simulation. -> **Status: MITIGATED (License Acquired Feb 11)**. |
| R2 | **Learning Curve Uncertainty** | 5 | 4 | **20** | There is no way to predict how long it will take to learn IPG CarMaker. Failure to master it quickly delays Phase 3 integration. **Mitigation:** Dedicate Week 4 entirely to tutorials. Use existing FSAI templates if available. Accept that initial simulations may be simple. |
| R3 | **Specific Hardware Failure** | 3 | 5 | **15** | The code is hardware-specific (Jetson AGX Orin). If the board is damaged, work stops. **Mitigation:** Treat hardware with care. Maintain a backup dev environment on Linux laptop. |

### Medium Priority Risks (Score 10-14)

| ID | Risk Description | Likelihood | Severity | Score | Mitigation Strategy |
|:---|:---|:---:|:---:|:---:|:---|
| R4 | **Data Loss** | 3 | 4 | **12** | Working on a single machine is a single point of failure. **Mitigation:** Strict Git usage. Remote backups daily. |
| R5 | **Health / Unforeseen Events** | 3 | 4 | **12** | Solo project dependency. **Mitigation:** Build "buffer weeks" (Week 6/12) to absorb downtime. |
| R6 | **Integration Failure (ROS 2 Bridge)** | 4 | 3 | **12** | The ROS 2 Bridge for CarMaker is notoriously tricky. Risk that Week 5 is spent fighting the interface instead of testing perception. **Mitigation:** Start integration early (Week 4 parallel with learning). Use Simple/Template projects first to validate data flow. |
| R7 | **Technical Knowledge Gap** | 4 | 3 | **12** | Risk of encountering complex technical concepts (e.g., advanced math, obscure errors) that require significant time to research. **Mitigation:** Allocate 20% of weekly time for "Research & Learning". Don't be afraid to ask for help/simplify the solution if blocked for >2 days. |

## Conclusion

This assessment highlights that the biggest threats to the project are **not technical bugs**, but **dependencies on specific tools and hardware**. By acknowledging this, I can plan around them—for example, by ensuring my code is modular enough to run on recorded data if the simulation license (R1) fails again.
