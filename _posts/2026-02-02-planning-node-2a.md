---
title: "Planning Node 2a: Cone Detection with YOLO"
date: 2026-02-02 09:00:00 +0000
categories: [Planning, Week 3]
tags: [yolo, cone-detection, planning, node-2a, dataset]
pin: false
---

Node 1a is working. Video footage is flowing through the pipeline at 30 Hz. Now it's time to teach the system what a cone actually looks like.

Today's goal: plan the implementation of **Node 2a**, the cone detection node that will take camera images and output bounding boxes around every cone it sees.

## The YOLO Decision (Revisited)

Back on January 27, I decided to use **YOLOv8** as the baseline detector instead of jumping straight into experimental models like YOLO26. That decision still holds.

YOLOv8 is stable, well-documented, and proven to work on the Jetson AGX Orin. More importantly, it has a massive community, which means when I inevitably run into issues, there's a good chance someone has already solved them.

The plan is simple: get YOLOv8 working first, establish a performance baseline, and *then* experiment with newer models if time permits.

## Dataset Research

For YOLO to detect cones, it needs to be trained on cone images. I spent some time researching available datasets and found the **[Formula Student Objects in Context (FSOCO)](https://fsoco.github.io/fsoco-dataset/)** dataset.

FSOCO is specifically designed for Formula Student autonomous racing. What makes it valuable is the diversity of conditions it covers:
- **Multiple tracks** from different competitions (FSG, FS Austria, FS East, etc.)
- **Varying weather conditions** (sunny, overcast, rain)
- **Different lighting** (bright daylight, shadows, late afternoon)
- **Real-world noise** (motion blur, occlusions, partial cone visibility)

The dataset includes thousands of labeled images with four cone classes:
- **Orange cones** (track boundaries)
- **Blue cones** (left side markers)
- **Yellow cones** (right side markers)
- **Large orange cones** (special markers)

Each image comes with bounding box annotations, which is exactly what YOLO needs for training.

I need to:
1. Download the FSOCO dataset
2. Verify the annotations are correct
3. Split into training/validation sets (typically 80/20)
4. Configure YOLOv8 for the four cone classes

## Node 2a Architecture

Node 2a will sit between Node 1a (camera publisher) and Node 3 (3D localizer):

```
Node 1a (Camera) → /camera/image_raw → Node 2a (Detector) → /detections → Node 3 (Localizer)
```

**Inputs:**
- Subscribe to `/camera/image_raw` (RGB images from Node 1a)

**Processing:**
- Run YOLO inference on each frame
- Extract bounding boxes, class labels, and confidence scores

**Outputs:**
- Publish detections to `/detections` topic
- Each detection includes: `[x, y, width, height, class, confidence]`

The node needs to run at ~30 Hz to keep up with the camera feed. YOLOv8 should be fast enough on the Jetson, but I'll need to verify this during testing.

## Next Steps

Tomorrow, I'll train the YOLO model on the FS-AI dataset. The training process will take a few hours, so I'll monitor the metrics (mAP, precision, recall) to ensure the model is learning properly.

Once the model is trained and validated, I'll integrate it into Node 2a on Wednesday and test the full pipeline end-to-end.

For now, the plan is clear. Time to start training.
