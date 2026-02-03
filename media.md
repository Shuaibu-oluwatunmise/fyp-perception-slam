# Blog Post Media Requirements

This document tracks all media (images, videos, screenshots) needed for each blog post.

---

## Jan 25: Week 1 - Project Selection
**Status:** ✅ Complete

**Media:**
- Gantt chart image (already embedded)

---

## Jan 26: Perception & SLAM System Explained
**Status:** ✅ Complete

**Media:**
- None required (text-only post)

---

## Jan 27: Technical Research Findings
**Status:** ✅ Complete

**Media:**
- None required (text-only post)

---

## Jan 28: The Strategy - A 3-Phase Approach to Perception
**Status:** ✅ Complete

**Media:**
- None required (text-only post)

---

## Jan 29: Anticipated Challenges & Development Setup
**Status:** ✅ Complete

**Media:**
- ✅ HIL setup photo (`/assets/img/blog/2026-01-29/hil_setup.jpg`)

---

## Jan 30: Building Node 1a - The Video Publisher
**Status:** 🔄 Needs Media

**Media Required:**
1. **Demo video** (YouTube embed)
   - Current: Placeholder using FSOCO Chalmers video (`id='xi28kF3kCN8'`)
   - **TODO:** Record actual demo showing:
     - Terminal with Node 1a running
     - `rqt_image_view` displaying video feed
     - `ros2 topic hz /camera/image_raw` showing ~30 Hz
     - Optional: Switch view to show mock depth visualization
   - Duration: 15-30 seconds
   - Upload to YouTube and replace video ID in post

**Alternative (if video not ready):**
- Screenshot of `rqt_image_view` showing video feed
- Screenshot of `rqt_image_view` showing mock depth
- Save to: `/assets/img/blog/2026-01-30/`

---

## Feb 2: Planning Node 2a - Cone Detection with YOLO
**Status:** ✅ Complete (no media needed)

**Media:**
- None required (planning/research post)

---

## Feb 3: Training YOLO for Cone Detection
**Status:** ⏳ Pending (post not written yet)

**Media Required:**
- TBD (will depend on training results documentation)
- Likely: Training metrics graphs, confusion matrix, sample detections

---

## Feb 4: Building Node 2a - Implementing Cone Detection
**Status:** ⏳ Pending (post not written yet)

**Media Required:**
- TBD (will likely need demo of detection working)
- Likely: Video/screenshots of YOLO detecting cones in real-time

---

## Summary

**Immediate Action Items:**
1. Record Node 1a demo video for Jan 30 post
2. Upload to YouTube and update video ID in post
3. Prepare training metrics/graphs for Feb 3 post (when ready)

**Media Storage:**
- Images: `/assets/img/blog/YYYY-MM-DD/`
- Videos: Upload to YouTube, embed with `{% include embed/youtube.html id='VIDEO_ID' %}`
