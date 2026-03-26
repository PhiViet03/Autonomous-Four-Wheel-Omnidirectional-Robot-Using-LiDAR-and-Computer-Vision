# 📡 LIDAR — Obstacle Avoidance Module

This folder contains the obstacle avoidance logic for the robot, using encoder feedback and motor control to physically dodge obstacles.

> ⚠️ Note: Despite the folder name, these files do **not** directly read from the LIDAR sensor. They handle the **movement response** when an obstacle is detected — the actual LIDAR sensor detection is handled in the main integration file.

---

## 📁 Files

### `6_DodgeTest.py` — Standalone Dodge Test
A **pure movement test** to verify that the robot can physically perform an obstacle avoidance maneuver, without any camera or LIDAR input.

**What it does:**
- Moves the robot **left → forward → right** to simulate dodging an obstacle
- Uses encoder odometry to measure exact distances moved
- Uses a PI controller per wheel for smooth and accurate movement
- Useful for tuning and validating the dodge sequence in isolation

**How to run:**
```bash
python3 6_DodgeTest.py
```
The robot will automatically execute the dodge sequence once and stop.

---

### `7_DodgeMove.py` — Dodge + Line Following (Integration Ready)
A **combined module** that merges obstacle avoidance with green-line following via camera. This is designed to be merged into the main control code.

**What it does:**
- Follows a **green line** on the ground using camera + HSV color detection
- Uses a **PID controller** to keep the robot centered on the line
- When an obstacle is detected (simulated by pressing `a` key in test mode), it triggers the **dodge maneuver**
- After dodging, resumes line following automatically

**Key components:**
| Component | Role |
|---|---|
| Camera (C270) | Captures frames, detects green line via HSV mask |
| PID Controller | Adjusts steering to stay centered on the line |
| Encoder Feedback | Measures wheel speed for closed-loop dodge movement |
| Mecanum IK | Converts velocity commands to individual wheel PWM signals |

**How to run (test mode):**
```bash
python3 7_DodgeMove.py
```
- Press `a` to simulate an obstacle and trigger the dodge
- Press `q` to quit

---

## ⚙️ Dodge Sequence (Both Files)

```
Detect obstacle
      │
      ▼
Move LEFT (0.5m or timed)
      │
      ▼
Move FORWARD (short burst)
      │
      ▼
Move RIGHT (return to lane)
      │
      ▼
Resume normal movement
```

---

## 🔧 Hardware Used
- Raspberry Pi 5
- Mecanum Wheels (60mm)
- DC Servo Motor GA12 N20
- H-Bridge L298N
- Wheel Encoders (quadrature)

---

## 📌 Notes
- `6_DodgeTest.py` is for **testing only** — it runs the dodge once and exits
- `7_DodgeMove.py` is the **production-ready** version meant to be integrated with the full system
- Encoder-based odometry is used for precise distance control during dodging