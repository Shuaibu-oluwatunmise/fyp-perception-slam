---
title: "New Home, New Problems: Installing CarMaker on Ubuntu"
date: 2026-02-23 18:00:00 +0000
categories: [Week 6, Phase 3]
tags: [ipg-carmaker, linux, ubuntu, troubleshooting, nvidia, xorg, wayland]
pin: false
image:
  path: /assets/img/thumbnails/23.png
  alt: Linux Troubleshooting and Setup
  header: false
---

So yeah, late last week, I made the call to pivot the whole perception architecture to Linux (Ubuntu 22.04 LTS). The custom Windows bridge I had built was just structurally incapable of parsing the complex beam tables coming out of CarMaker's LiDAR simulation. 

Honestly, I really only had two choices here: spend days (or maybe weeks) trying to reverse-engineer the beam table math in Python just to recreate a point cloud, or switch to an environment where IPG's official `CMRosIF` (CarMaker ROS Interface) just works natively. I chose the latter. It felt like the actual professional path forward.

I had hoped the Linux license would arrive on Monday, and thankfully, it actually did. I woke up to a message from Ahsan (president of the FSAI at Middlesex University) confirming the new license was generated. Pretty nice morning, to be honest.

But if you've ever tried installing highly specialized engineering software on Linux, you know that "smooth sailing" isn't really a thing.

Spoiler: the day ended well.

{% include embed/youtube.html id='YTDPAY9WHic' %}
_CarMaker running natively on Ubuntu 22.04 with full hardware acceleration — this took some doing._

But the struggles were real. Here's how we got there.

## The Installation (The "Easy" Part)

Setting up the new environment meant starting from scratch. First, I had to install Ubuntu 22.04 LTS on a new partition so I could dual-boot. Once the OS was up, I grabbed **ROS 2 Humble**—adding the repository keys, running `apt upgrade`, and getting `ros-humble-desktop` installed and sourced in my `.bashrc`. It's basically a rite of passage for robotics at this point, but surprisingly, it went pretty smoothly.

Next came IPG CarMaker. I downloaded the Linux installer and the Formula Student package from the customer area. Since I had just gone through this whole process on Windows, the Linux install felt strangely familiar. 

I kept a copy of the official Linux installation guide for reference if anyone else needs it:

📄 [IPG CarMaker Linux Installation Guide](/fyp-perception-slam/assets/docs/blog/2026-02-23/InstallationGuide.pdf)

I extracted the zip, ran `sudo ./ipg-install`, and added the binaries to my PATH. Then, just like on Windows, you have to strip the file extension off the license file and drop it into `/opt/ipg/etc/Licenses`.

![Terminal view showing the installed CarMaker license file in the correct directory](/assets/img/blog/2026-02-23/Liscences.png)
_The license file sitting right where it should be in `/opt/ipg/etc/Licenses`._

I rebooted the system, opened a terminal, and typed `CM`. The CarMaker GUI popped right up. Feeling pretty confident, I loaded up the FS_autonomous project and hit the button to launch **MovieNX**—CarMaker's high-end 3D rendering engine.

And that's when reality hit.

## Hurdle 1: The Wayland Wall

Instead of seeing a 3D track, my terminal just spat out a critical Vulkan error:

```bash
Movie NX #0: [CRITICAL ERROR] [Engine] VkExt::init(): Failed to initialize GPU Adapter - GPU Adapter doesn't have additional queue family with transfer
```

This is one of those errors that looks terrifying but actually has a very specific cause. Ubuntu 22.04 defaults to using **Wayland** as its display server. While Wayland is the new standard, MovieNX's rendering pipeline relies heavily on Vulkan GPU queue stuff that Wayland just doesn't support well yet. 

The fix wasn't even a code change; it was just an OS config change. I had to log out, click the little gear icon on the Ubuntu login screen, and switch my session from Wayland over to **"Ubuntu on Xorg"**. 

## Hurdle 2: The Graphics Driver Check

I also knew from past projects that robotics simulations and integrated graphics do not mix. Even though I was now running on Xorg, I wanted to make sure CarMaker actually had access to the NVIDIA RTX A2000 in my laptop before pushing it.

I ran `nvidia-smi` to check my GPU loads, and the terminal basically laughed at me: `Command 'nvidia-smi' not found`.

My brand-new Ubuntu installation was running on the default, open-source `nouveau` drivers. Without the proprietary NVIDIA drivers, I had no CUDA support and no hardware acceleration for Vulkan. Trying to run a physics simulation and 3D rendering engine on integrated graphics is like trying to race a Formula Student car with a lawnmower engine. 

To fix it, I just used `ubuntu-drivers autoinstall` to pull down the recommended `nvidia-driver-590-open` package. After a quick reboot, I ran `prime-select nvidia` to force the system into discrete-graphics-only mode.

## Looking Ahead

By the end of the day, I finally had a stable, natively accelerated CarMaker environment running on Ubuntu 22.04. 

The difference between this and the hacked-together Windows TCP bridge I had before is night and day. It's faster, the rendering is flawless, and most importantly, it's the environment the Formula Student AI package was *actually built* to run in.

With the environment finally stable, tomorrow's goal is to load up that `FS_autonomous` package and look at how the ROS interface is structured under the hood. If everything goes to plan, `CMRosIF` should be publishing sensor data directly as native ROS topics — meaning `/camera/image_raw`, `/lidar/pointcloud`, and depth data all flowing cleanly into the same pipeline we already built, no custom TCP bridges, no manual byte parsing. Just clean, timestamped ROS messages. That alone would be a massive step forward.
