---
title: "RSDS, Sensor Clusters, and Building the CarMaker Bridge"
date: 2026-02-16 18:00:00 +0000
categories: [Week 5, Phase 3]
tags: [ipg-carmaker, rsds, sensor-cluster, simulation, fs-autonomous, bridge]
pin: false
image:
  path: /assets/img/thumbnails/18.png
  alt: CarMaker RSDS Bridge
  header: false
---

Today was one of those days where you sit down to read documentation and end up building something by the end of it.

The Reference Manual I started reading on Friday pointed me to a concept called **RSDS** — the **Raw Sensor Data Stream**. That one concept unlocked most of what I spent today doing: finding IPG's example code, loading a Formula Student track, configuring sensor clusters, and getting real simulation data flowing out of CarMaker for the first time.

Here's all of it in one take:

{% include embed/youtube.html id='5MCQXAp3qFc' %}
_Full walkthrough: RSDS, the FS_autonomous package, sensor cluster setup, and the bridge working end-to-end._

---

## What is RSDS?

**RSDS** stands for **Raw Sensor Data Stream**. It's the mechanism CarMaker uses to stream simulated sensor data — camera frames, LiDAR point clouds — out of the simulation to external client processes over TCP.

By default, CarMaker sends all GPU sensor results back to its own simulation program. But with RSDS, you can define an *external* client as the receiver instead. That client connects to a port, listens, and receives the raw data in real time.

The Reference Manual covers this in Chapter 21's GPU Sensor Framework (section 21.3.7). Crucially, it also pointed me to example client code buried in CarMaker's installation `examples/` directory — `rsds-client.c` — which shows the basic structure for receiving and parsing RSDS messages. That became the foundation of everything I built today.

---

## The Formula Student CarMaker Package

While digging through the IPG Customer Area, I found something useful: an official **Formula Student CarMaker package** — a pre-built CarMaker project with track scenarios, cone placements, and a Formula Student car model. Exactly what I needed to stop testing on a generic road and start testing on something that looks like an actual FS track.

I downloaded it and moved the `FS_autonomous` folder into `C:\CM_Projects` — the directory CarMaker uses to find projects. Once there, it appeared in the CarMaker UI and I could load it directly.

---

## The Errors (And What They Tell You)

Loading the project wasn't perfectly clean. Two popups showed up immediately:

![Road definition error](/assets/img/blog/2026-02-16/first_error.png)
_Error 1: Road definition failed to load. Safe to dismiss — the project still loads._

This one is safe to ignore. The road geometry doesn't fully load (likely a file path issue with how the package was built), but the project itself opens and runs well enough to work with.

The second one was more telling:

![ROS launch error](/assets/img/blog/2026-02-16/second_error.png)
_Warning: CarMaker tried to call `::CMRosIF::ROS_Launch` — a ROS function that doesn't exist on Windows._

That warning is exactly what it looks like. The `FS_autonomous` package includes a `.CarMaker.tcl` GUI extension that tries to call a ROS launch command on startup. On a Linux machine with the CarMaker ROS interface installed, this would work fine. On Windows, it fails silently with a warning.

This confirmed something I had already suspected: **IPG expects Formula Student AI teams to use ROS**. The package was built for a Linux/ROS workflow.

---

## The Linux Licence Application

My Windows machine is set up for dual boot. I already had the CarMaker licence for Windows — tied to the hostname, MAC address, and OS. But if Windows proves difficult to work around (and this warning suggested it might), I'd want the option to switch to Linux and run a proper ROS stack there.

So I put in an application for a second licence — same MAC address, Linux OS, slightly different hostname (dual boot produces a different hostname). This way the option is open if I need it. For now, I'm continuing on Windows.

---

## Finding the Bridge Code

Section 21.3.7 of the Reference Manual pointed explicitly to example RSDS client code inside CarMaker's installation directory. I found it, read through it, and it showed the full structure: how to connect to CarMaker's RSDS server over TCP, read the 64-byte ASCII header from each message, and parse the binary payload for each sensor type.

I spent time playing with these files, understanding what each one did, and then organised everything into a dedicated repository:

**[carmaker-rsds-bridge](https://github.com/Shuaibu-oluwatunmise/carmaker-rsds-bridge)**

The repo contains the compiled EXE clients and the Python scripts needed to launch them and process their output. Here's the key breakdown:

| File | What it does |
|---|---|
| `rsds-client-camera-standalone.exe` | Connects to a camera port, saves frames as `.ppm` files |
| `rsds-client.exe` | Connects to the LiDAR port, prints point cloud data to console |
| `start_rsds_clients.py` | Launches all three client instances simultaneously |
| `convert_ppm_to_png.py` | Batch converts `.ppm` output to viewable `.png` files |

The camera client is run **twice** — once on port `2210` for RGB frames and again on port `2211` for depth frames. The LiDAR client runs separately on port `2212`. Three client instances total.

---

## Configuring the Sensor Cluster

For data to actually stream out, CarMaker needs to know what to send and where. This is done through the **Sensor Cluster** configuration — CarMaker's way of grouping sensors that share a GPU render pass and defining their output ports.

| Sensor | Port |
|---|---|
| Camera RGB | 2210 |
| Camera Depth | 2211 |
| LiDAR | 2212 |

If the LiDAR sensor cluster isn't configured to broadcast on port `2212`, CarMaker won't attempt to stream LiDAR data at all — you'll get a `RSDS client not started` warning in the session log when the simulation runs.

One important setting: **VIB mode must be OFF** on the camera sensors. If it's enabled, no frames will be received regardless of whether the client is connected.

I also reduced the resolution of both the RGB and depth cameras to **640×480** in the sensor configuration. The full-resolution output was putting unnecessary load on the GPU. Dropping it to 640×480 keeps performance stable and still gives us plenty of detail for the perception pipeline.

---

## The GPU Timeout: Order Matters

This one cost me some time. When I ran a simulation without the RSDS clients already connected and listening, this appeared in the session log:

![Session log showing GPU timeout](/assets/img/blog/2026-02-16/third_error.png)
_The sequence: RSDS client not running (×2) → GPUSensor Timeout → SIM_ABORT._

CarMaker's GPU sensors time out if nothing is connected to receive their data. The simulation aborts entirely.

**The correct order:**
1. Run `start_rsds_clients.py` first — all three client windows open and wait for a connection
2. Then start the simulation

Once the clients are running, CarMaker connects to them and data starts flowing immediately.

---

## Running the Clients

`start_rsds_clients.py` handles the setup in one go. It opens three separate PowerShell windows — one per sensor — each running the correct client executable with the right port flags:

```powershell
python start_rsds_clients.py
```

That's it. Three windows open, each waiting for CarMaker to start streaming. When the simulation starts, data appears immediately: camera frames start saving to `data/camera/raw/` as `.ppm` files, and LiDAR point counts appear in the terminal in real time.

---

## Output: PPM → PNG

Camera frames save as `.ppm` files — a raw image format that most image viewers can't open. To view them, run the conversion script:

```powershell
python convert_ppm_to_png.py
```

It batch-converts everything in `data/camera/raw/` to `.png` files in `data/camera/processed/`. The first time you see a real simulation frame rendered from CarMaker and converted to a viewable image, it feels like something actually working.

LiDAR data appears directly in the terminal — point counts per scan, printed as they arrive. File saving for LiDAR is planned for a later stage.

---

## Where Things Stand

The bridge is working. Simulation data — camera and LiDAR — is flowing out of CarMaker and being captured on the Windows machine. The next challenge is getting that data across to the Jetson, which is where the ROS 2 perception pipeline lives.

That's the next post.
