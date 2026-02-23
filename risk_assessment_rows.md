# PDE3823 Risk Assessment — Row Guide

**Project Title:** Perception and SLAM System for Autonomous Formula Student Racing
**Project Owner:** [Your name]
**Responsible Person:** [Supervisor name]
**Start Date:** January 2026 | **End Date:** May 2026

---

## Row 1 — Computing Equipment

| Field | Value |
|---|---|
| Task / Activity | Operating PC and computing hardware during development |
| Hazard | Electrical fault from damaged cable or overloaded socket |
| Who Might Be Harmed | Student |
| Risk Likelihood | Very Unlikely |
| Risk Severity | Minor Injury |
| Risk Level | 2 |
| Control Measures | PAT-tested lab equipment; cable integrity checked; no modifications to power hardware |
| New Risk Likelihood | Very Unlikely |
| New Risk Severity | Near Miss |
| Controlled Risk Level | 1 |

---

## Row 2 — GPU Overheating

| Field | Value |
|---|---|
| Task / Activity | Running GPU-intensive CarMaker simulation sessions |
| Hazard | PC/GPU overheating causing hardware failure |
| Who Might Be Harmed | Student |
| Risk Likelihood | Unlikely |
| Risk Severity | Near Miss |
| Risk Level | 2 |
| Control Measures | Adequate ventilation; GPU temperature monitored; camera resolution reduced to 640×480; session length managed |
| New Risk Likelihood | Very Unlikely |
| New Risk Severity | Near Miss |
| Controlled Risk Level | 1 |

---

## Row 3 — Prolonged Screen Use

| Field | Value |
|---|---|
| Task / Activity | Extended development and debugging sessions |
| Hazard | Eye strain or RSI from sustained screen use |
| Who Might Be Harmed | Student |
| Risk Likelihood | Possible |
| Risk Severity | Minor Injury |
| Risk Level | 6 |
| Control Measures | Regular breaks taken; ergonomic seating; 20-20-20 rule followed |
| New Risk Likelihood | Unlikely |
| New Risk Severity | Near Miss |
| Controlled Risk Level | 2 |

---

## Row 4 — Licence / Tool Dependency

| Field | Value |
|---|---|
| Task / Activity | Simulation development using IPG CarMaker |
| Hazard | CarMaker licence tied to specific hardware — machine failure blocks all simulation access |
| Who Might Be Harmed | Student (project impact) |
| Risk Likelihood | **Likely** |
| Risk Severity | Near Miss |
| Risk Level | 4 |
| Control Measures | Second licence applied for (Linux/dual boot, same MAC); mock data pipeline retained as fallback; supervisor informed |
| New Risk Likelihood | Unlikely |
| New Risk Severity | Near Miss |
| Controlled Risk Level | 2 |

---

## Row 5 — Faulty Algorithm Output

| Field | Value |
|---|---|
| Task / Activity | Deploying cone detection and localisation algorithm |
| Hazard | False detections or missed cones causing incorrect navigation decisions in real FS-AI deployment |
| Who Might Be Harmed | All persons onsite (competition participants, marshals) |
| Risk Likelihood | Possible |
| Risk Severity | Major Injury |
| Risk Level | 12 |
| Control Measures | Extensive simulation testing before real-world use; conservative confidence thresholds; system documented as prototype; human oversight maintained |
| New Risk Likelihood | Unlikely |
| New Risk Severity | Minor Injury |
| Controlled Risk Level | 4 |

---

## Row 6 — Misleading Simulation Results

| Field | Value |
|---|---|
| Task / Activity | Validating perception pipeline in CarMaker simulation |
| Hazard | Simulation not fully representative of real-world conditions → overconfidence in system performance |
| Who Might Be Harmed | Student / downstream decision makers |
| Risk Likelihood | Possible |
| Risk Severity | Minor Injury |
| Risk Level | 6 |
| Control Measures | All simulation assumptions documented; outputs labelled as synthetic; no real-world deployment claims without physical validation |
| New Risk Likelihood | Unlikely |
| New Risk Severity | Near Miss |
| Controlled Risk Level | 2 |

---

## Row 7 — Training Data Bias

| Field | Value |
|---|---|
| Task / Activity | Training YOLOv8 cone detector on FSOCO dataset |
| Hazard | Dataset underrepresentation of cone types/lighting → poor detection in edge cases |
| Who Might Be Harmed | All persons onsite (real deployment) |
| Risk Likelihood | Possible |
| Risk Severity | Minor Injury |
| Risk Level | 6 |
| Control Measures | Dataset composition reviewed; precision-first threshold tuning; known failure modes documented (distance limitations) |
| New Risk Likelihood | Unlikely |
| New Risk Severity | Near Miss |
| Controlled Risk Level | 2 |

---

## Row 8 — Data Loss / Software Failure

| Field | Value |
|---|---|
| Task / Activity | Storing training data, simulation outputs, and code |
| Hazard | Accidental data loss or corruption causing project setback |
| Who Might Be Harmed | Student |
| Risk Likelihood | Unlikely |
| Risk Severity | Near Miss |
| Risk Level | 2 |
| Control Measures | All code version-controlled on GitHub; simulation data backed up; training results archived locally and remotely |
| New Risk Likelihood | Very Unlikely |
| New Risk Severity | Near Miss |
| Controlled Risk Level | 1 |
