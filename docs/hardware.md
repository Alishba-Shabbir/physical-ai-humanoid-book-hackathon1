---
sidebar_position: 6
---

# Hardware Requirements

This page lists recommended hardware for the Physical AI & Humanoid Robotics course. The recommendations are grouped by use case: development workstations for digital twins, an edge kit for field deployment, and lab platform options.

## 1. Digital Twin Workstation

- GPU: NVIDIA RTX 4070 Ti (12 GB VRAM) or higher
- CPU: Intel Core i7 13th Gen+ or AMD Ryzen 9
- RAM: 64 GB DDR5 (minimum 32 GB)
- OS: Ubuntu 22.04 LTS
- Note: Required for running Isaac Sim, Gazebo, and Unity smoothly. Use an NVMe SSD (1 TB+) for datasets and scenes.

## 2. Physical AI Edge Kit

- Brain: NVIDIA Jetson Orin Nano 8GB or Orin NX 16GB
- Vision: Intel RealSense D435i or D455 (RGB + Depth + IMU)
- Balance: USB IMU (BNO055)
- Voice Interface: USB microphone and speaker
- Note: Used to deploy ROS 2 nodes and test AI models physically.

## 3. Robot Lab Options

- **Option A:** Unitree Go2 Edu (Budget Proxy)
  - Pros: Durable, affordable, good ROS 2 support
  - Cons: Not humanoid

- **Option B:** Miniature Humanoid (Hiwonder TonyPi Pro / Robotis OP3)
  - Pros: Table-top humanoid, affordable
  - Cons: Limited AI processing

- **Option C:** Premium Lab: Unitree G1 Humanoid
  - Pros: Real bipedal movement, full SDK access
  - Cons: Expensive

### Notes

- Use proper Markdown headings (`#`, `##`, `###`) and bullets (`-`) for all sub-items.
- Ensure there are no lines starting with raw numbers without list markers to avoid Docusaurus compilation errors.
- Make it readable, professional, and visually clean.

---

If you would like, I can add an installation checklist (Ubuntu 22.04 LTS, ROS 2, NVIDIA drivers, Isaac Sim) as a follow-up.
