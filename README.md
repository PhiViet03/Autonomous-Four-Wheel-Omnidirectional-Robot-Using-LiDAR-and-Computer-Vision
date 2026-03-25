# 🚗 Autonomous Mecanum Robot with LIDAR & Computer Vision

A 4-wheeled omnidirectional autonomous robot that integrates LIDAR sensing and camera-based traffic sign recognition for intelligent navigation and obstacle avoidance.

---

## 📌 Overview

This project addresses a core challenge in autonomous vehicle systems: **no single sensor is sufficient on its own.**

- **LIDAR** is accurate for 3D mapping and obstacle detection — but lacks semantic understanding.
- **Camera + CV** can recognize traffic signs — but is sensitive to lighting and environment.

By fusing both, this robot can simultaneously **avoid obstacles** and **obey traffic rules** in real time.

---

## 🎯 Objectives

- Design and build a smart 4-wheeled Mecanum robot
- Integrate LIDAR (C1M1) for spatial scanning and obstacle avoidance
- Use a camera (C270) + YOLOv8n model for real-time traffic sign recognition
- Deploy the full pipeline on a Raspberry Pi 5 for edge inference

---

## 🛠️ Hardware Components

| Component | Role |
|---|---|
| Raspberry Pi 5 | Central processing unit |
| LIDAR C1M1 | Spatial scanning & obstacle detection |
| Logitech C270 Camera | Traffic sign image capture |
| Mecanum Wheels (60mm) | Omnidirectional movement |
| DC Servo Motor GA12 N20 | Wheel actuation |
| H-Bridge L298N | Motor driver |
| Li-ion 18650 Battery | Power supply |
| Buck Converter XL4015 | Voltage regulation |

---

## 💻 Software Stack

- **YOLOv8n** — Real-time traffic sign detection and classification
- **LIDAR data processing** — Obstacle mapping and safe-distance maintenance
- **Motor control algorithm** — Translates sensor decisions into wheel commands
- All deployed on **Raspberry Pi 5** for real-time edge processing

---

## 🔄 System Architecture
```
Camera Feed ──► YOLOv8n ──────────────────────┐
                                               ▼
                                     Decision Engine (RPi 5)
                                               │
LIDAR Scan ──► Obstacle Map ─────────────────►│
                                               ▼
                                    L298N Motor Driver
                                               │
                                     Mecanum Wheel Drive
```

---

## ✅ Key Features

- **Omnidirectional movement** via Mecanum wheels
- **Real-time obstacle avoidance** using LIDAR point cloud data
- **Traffic sign recognition** (stop, speed limit, turn signs, etc.) via YOLOv8
- **Sensor fusion** — camera and LIDAR work together for robust decision-making
- **Compact & low-cost** — fully runs on Raspberry Pi 5

---

## ⚠️ Limitations

- Tested in a **controlled small-scale environment only**; not validated in real traffic
- Camera performance degrades in **low-light or harsh weather** conditions
- LIDAR cost and outdoor durability remain areas for improvement
- Processing is constrained by **Raspberry Pi 5 hardware capacity**

---

## 🌐 Practical Applications

- Automated goods transport in warehouses or factories
- AI-powered smart traffic systems
- Research and education platform for autonomous vehicle development

---

## 🔭 Scope

This project is scoped to a **model-scale test vehicle** in a pre-designed environment. Real-world traffic conditions, diverse weather, and high-speed scenarios are outside the current research scope.

---

## 📚 Theoretical Background

- Computer Vision & Deep Learning (YOLOv8)
- Mobile Robot Control & Navigation
- LIDAR Sensor Technology
- Obstacle Avoidance & Sign-Based Control Algorithms
