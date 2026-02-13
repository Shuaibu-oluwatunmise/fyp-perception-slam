---
layout: page
title: Literature Review
icon: fas fa-book
order: 2
---

<style>
details {
    background-color: var(--card-bg);
    border-radius: 10px;
    margin-bottom: 30px;
    border: 1px solid var(--card-border);
    transition: all 0.3s ease;
    box-shadow: 0 4px 6px rgba(0,0,0,0.1);
}

details:hover {
    box-shadow: 0 8px 12px rgba(0,0,0,0.2);
    border-color: var(--link-color);
}

summary {
    cursor: pointer;
    padding: 25px;
    font-size: 1.4rem;
    font-weight: 600;
    list-style: none;
    display: flex;
    justify-content: space-between;
    align-items: center;
    color: var(--heading-color);
}

/* Fix for the square: Use a unicode character or simple CSS shape instead of FA font dependency */
summary::after {
    content: '▼'; 
    font-size: 0.8em;
    transition: transform 0.3s ease;
    opacity: 0.7;
}

details[open] summary::after {
    transform: rotate(180deg);
}

/* Remove default triangle in Webkit */
summary::-webkit-details-marker {
    display: none;
}

.content-box {
    padding: 0 25px 25px 25px;
    color: var(--text-color);
}

.paper-item {
    padding: 20px 0;
    border-bottom: 1px solid var(--border-color);
}
.paper-item:last-child {
    border-bottom: none;
}
.paper-citation {
    font-family: "Times New Roman", Times, serif; /* IEEE style font */
    font-size: 1.05rem;
    color: var(--heading-color);
    margin-bottom: 8px;
    display: block;
}
.paper-links {
    margin-bottom: 10px;
    font-size: 0.9rem;
}
.paper-relevance {
    padding: 12px;
    background-color: rgba(0,0,0,0.03);
    border-radius: 6px;
    border-left: 3px solid var(--link-color);
    font-size: 0.95rem;
}

</style>

<div class="literature-container">

<details>
<summary>1. Perception & Sensor Fusion</summary>
<div class="content-box">

<!-- [1] Perception > Fusion -->
<div class="paper-item">
    <span class="paper-citation">[1] J. Fayyad, M. A. Jaradat, D. Gruyer, and H. Najjaran, "Deep Learning Sensor Fusion for Autonomous Vehicle Perception and Localization: A Review," <em>Sensors</em>, vol. 20, no. 15, p. 4220, 2020.</span>
    <div class="paper-links">
        <a href="https://www.mdpi.com/1424-8220/20/15/4220" target="_blank"><i class="fas fa-external-link-alt"></i> Read Paper</a>
    </div>
    <div class="paper-relevance">
        <strong>Relevance:</strong> Foundational for designing the <strong>sensor fusion architecture</strong>. Justifies the choice of <strong>Late Fusion</strong> (detecting then fusing) over Early Fusion.
    </div>
</div>

<!-- [2] Perception > Dataset -->
<div class="paper-item">
    <span class="paper-citation">[2] N. Vödisch, D. Dodel, and M. Schötz, "FSOCO: The Formula Student Objects in Context Dataset," <em>SAE International Journal of Connected and Automated Vehicles</em>, vol. 5, 2022.</span>
    <div class="paper-links">
        <a href="https://fsoco.github.io/fsoco-dataset/" target="_blank"><i class="fas fa-database"></i> View Dataset</a> | 
        <a href="https://arxiv.org/abs/2012.07139" target="_blank"><i class="fas fa-file-pdf"></i> Read Paper</a>
    </div>
    <div class="paper-relevance">
        <strong>Relevance:</strong> The primary training dataset for Formula Student. Essential for training the <strong>cone detection</strong> model.
    </div>
</div>

<!-- [3] Perception > YOLO Theory -->
<div class="paper-item">
    <span class="paper-citation">[3] "Real-Time Object Detection in Autonomous Vehicles with YOLO," 2024.</span>
    <div class="paper-links">
        <a href="https://www.sciencedirect.com/science/article/pii/S1877050924024293" target="_blank"><i class="fas fa-external-link-alt"></i> Read Paper</a>
    </div>
    <div class="paper-relevance">
        <strong>Relevance:</strong> Justifies the selection of <strong>YOLOv8</strong> for the vision pipeline due to its real-time performance optimization.
    </div>
</div>

<!-- [4] Perception > Jetson/Edge -->
<div class="paper-item">
    <span class="paper-citation">[4] "YOLO-Based Object Detection and Tracking for Autonomous Vehicles Using Edge Devices," 2022.</span>
    <div class="paper-links">
        <a href="https://www.researchgate.net/publication/365583892_YOLO-Based_Object_Detection_and_Tracking_for_Autonomous_Vehicles_Using_Edge_Devices" target="_blank"><i class="fas fa-external-link-alt"></i> Read Paper</a>
    </div>
    <div class="paper-relevance">
        <strong>Relevance:</strong> Implementation guide for deploying YOLO on **NVIDIA Jetson AGX**, addressing real-time constraints.
    </div>
</div>

<!-- [5] Perception > YOLO26 -->
<div class="paper-item">
    <span class="paper-citation">[5] Ultralytics, "YOLO26: Scalable Object Detection for Real-Time Applications," <em>Ultralytics Releases</em>, 2026.</span>
    <div class="paper-links">
        <a href="https://github.com/ultralytics/ultralytics" target="_blank"><i class="fab fa-github"></i> Official Repository</a>
    </div>
    <div class="paper-relevance">
        <strong>Relevance:</strong> The state-of-the-art <strong>perception model</strong> planned for future implementation. Offers potential for higher accuracy/speed trade-offs compared to v8.
    </div>
</div>

<!-- [13] Perception > YOLOv13-Cone-Lite -->
<div class="paper-item">
    <span class="paper-citation">[13] C. Zhang et al., "YOLOv13-Cone-Lite: An Enhanced Algorithm for Traffic Cone Detection in Autonomous Formula Racing Cars," <em>Applied Sciences</em>, vol. 13, 2023.</span>
    <div class="paper-links">
        <a href="https://www.mdpi.com/2076-3417/13/10/6122" target="_blank"><i class="fas fa-external-link-alt"></i> Read Paper</a>
    </div>
    <div class="paper-relevance">
        <strong>Relevance:</strong> A 2023 study benchmarking custom YOLO architectures specifically for FS cones. Useful for comparing our <strong>YOLOv8</strong> results against other custom models.
    </div>
</div>

</div>
</details>

<details>
<summary>2. SLAM Algorithms</summary>
<div class="content-box">

<!-- [6] SLAM > Toolbox -->
<div class="paper-item">
    <span class="paper-citation">[6] S. Macenski and I. Jambrecic, "SLAM Toolbox: SLAM for the Dynamic World," <em>Journal of Open Source Software</em>, vol. 6, no. 61, p. 2783, 2021.</span>
    <div class="paper-links">
        <a href="https://joss.theoj.org/papers/10.21105/joss.02783" target="_blank"><i class="fas fa-external-link-alt"></i> Read Paper</a>
    </div>
    <div class="paper-relevance">
        <strong>Relevance:</strong> Documentation for the ROS2 default SLAM package (Pose-graph optimization). Critical for tuning the <strong>mapping system</strong>.
    </div>
</div>

<!-- [7] SLAM > Cartographer -->
<div class="paper-item">
    <span class="paper-citation">[7] S. Y. Kim et al., "Improving Sensor Adaptability and Functionality in Cartographer Simultaneous Localization and Mapping," <em>Sensors</em>, vol. 25, no. 6, 2025.</span>
    <div class="paper-links">
        <a href="https://www.mdpi.com/1424-8220/25/6/1808" target="_blank"><i class="fas fa-external-link-alt"></i> Read Paper</a>
    </div>
    <div class="paper-relevance">
        <strong>Relevance:</strong> Explains graph-based SLAM architecture, providing a strong alternative comparison to SLAM Toolbox.
    </div>
</div>

<!-- [8] SLAM > Comparison -->
<div class="paper-item">
    <span class="paper-citation">[8] A. Kucuksubasi and A. B. Can, "From Simulation to Reality: Comparative Performance Analysis of SLAM Toolbox and Cartographer in ROS 2," <em>Electronics</em>, vol. 14, no. 24, 2025.</span>
    <div class="paper-links">
        <a href="https://www.mdpi.com/2079-9292/14/24/4822" target="_blank"><i class="fas fa-external-link-alt"></i> Read Paper</a>
    </div>
    <div class="paper-relevance">
        <strong>Relevance:</strong> Direct head-to-head comparison. Used to justify the final decision (likely SLAM Toolbox for ease of use).
    </div>
</div>

</div>
</details>

<details>
<summary>3. LiDAR Processing</summary>
<div class="content-box">

<!-- [9] LiDAR > Clustering Review -->
<div class="paper-item">
    <span class="paper-citation">[9] A. S. Khan et al., "Systematic and Comprehensive Review of Clustering and Multi-Target Tracking Techniques for LiDAR Point Clouds in Autonomous Driving Applications," <em>Sensors</em>, vol. 23, no. 13, 2023.</span>
    <div class="paper-links">
        <a href="https://www.mdpi.com/1424-8220/23/13/6119" target="_blank"><i class="fas fa-external-link-alt"></i> Read Paper</a>
    </div>
    <div class="paper-relevance">
        <strong>Relevance:</strong> Clustering (DBSCAN/K-Means) is the math behind recognizing cones in a point cloud. Essential for the <strong>LiDAR processing pipeline</strong>.
    </div>
</div>

<!-- [10] LiDAR > Small Object -->
<div class="paper-item">
    <span class="paper-citation">[10] C. Li, F. Gao, X. Han, and B. Zhang, "A Small-Object-Detection Algorithm Based on LiDAR Point-Cloud Clustering for Autonomous Vehicles," <em>Sensors</em>, vol. 24, no. 16, 2024.</span>
    <div class="paper-links">
        <a href="https://www.mdpi.com/1424-8220/24/16/5423" target="_blank"><i class="fas fa-external-link-alt"></i> Read Paper</a>
    </div>
    <div class="paper-relevance">
        <strong>Relevance:</strong> Solves the "distant cone problem" (fews points at range) using adaptive clustering radii.
    </div>
</div>

</div>
</details>

<details>
<summary>4. Formula Student AI Specifics</summary>
<div class="content-box">

<!-- [11] FS > Lane Detection -->
<div class="paper-item">
    <span class="paper-citation">[11] I. Ivanov and C. Markgraf, "Lane Detection using Graph Search and Geometric Constraints for Formula Student Driverless," <em>arXiv preprint</em> arXiv:2405.16369, 2024.</span>
    <div class="paper-links">
        <a href="https://arxiv.org/abs/2405.16369" target="_blank"><i class="fas fa-external-link-alt"></i> Read Paper</a>
    </div>
    <div class="paper-relevance">
        <strong>Relevance:</strong> Algorithms for handling "missing cones" using geometric constraints of the track.
    </div>
</div>

<!-- [12] FS > Rules -->
<div class="paper-item">
    <span class="paper-citation">[12] Institution of Mechanical Engineers (IMechE), "Formula Student AI Rules," 2024-2026.</span>
    <div class="paper-links">
        <a href="https://www.imeche.org/docs/default-source/1-oscar/formula-student/2024/rules/formula-student-2024-rules.pdf" target="_blank"><i class="fas fa-file-pdf"></i> Read Rules PDF</a>
    </div>
    <div class="paper-relevance">
        <strong>Relevance:</strong> Defines the <strong>System Requirements</strong> (ADS Class, DDT Class, Safety) that the project must strictly adhere to.
    </div>
</div>

<!-- [14] FS > Full System SLAM -->
<div class="paper-item">
    <span class="paper-citation">[14] "Perception system. Simultaneous Localization and Mapping for a Formula Student Driverless Race Car," <em>ReadyTensor Projects</em>, 2024.</span>
    <div class="paper-links">
        <a href="https://app.readytensor.ai/publications/perception-system-simultaneous-localization-and-mapping-for-a-formula-student-driverless-race-car-6T7XkF3b" target="_blank"><i class="fas fa-external-link-alt"></i> Read Project</a>
    </div>
    <div class="paper-relevance">
        <strong>Relevance:</strong> Validates our <strong>3-Phase Architecture</strong> (Detection → Localization → SLAM). Demonstrates a successful integration of YOLOv8 with a Visual SLAM framework.
    </div>
</div>

<!-- [15] FS > Vision Clustering -->
<div class="paper-item">
    <span class="paper-citation">[15] "Innovative Cone Clustering and Path Planning for Autonomous Formula Student Race Cars Using Cameras," <em>MDPI Sensors</em>, 2024.</span>
    <div class="paper-links">
        <a href="https://www.mdpi.com/1424-8220/24/5/1645" target="_blank"><i class="fas fa-external-link-alt"></i> Read Paper</a>
    </div>
    <div class="paper-relevance">
        <strong>Relevance:</strong> Discusses <strong>Vision-only Clustering</strong>. Important for justifying why we added LiDAR (to overcome the limitations of this exact camera-only approach).
    </div>
</div>

</div>
</details>

</div>