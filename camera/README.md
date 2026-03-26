# 📷 Camera — Line Following & Turn Module

This folder contains the camera-based vision pipeline for the robot, handling green line detection, lane following, and turn execution.

---

## 📁 Files

### `4_CameraTest.py` — Camera Stream Viewer
A **visual debugging tool** to verify the camera is detecting the green line correctly, streamed live to a browser.

**What it does:**
- Opens the camera and detects a **green line** on the ground using HSV color filtering
- Calculates the **offset** (how far the robot is from the center of the line)
- Streams the annotated video feed to a browser via Flask at `http://<raspberry-pi-ip>:5000/video`
- Draws the detected line center, robot center, and offset value on screen

**How to run:**
```bash
python3 4_CameraTest.py
```
Then open a browser and go to:
```
http://<your-raspberry-pi-ip>:5000/video
```

> ✅ Use this first to confirm the camera sees the line before running any movement code.

---

### `5_CamWheel.py` — Camera + Wheel Integration
Combines camera line detection with **actual motor control** — the robot physically follows the green line.

**What it does:**
- Detects the green line using HSV masking on the camera ROI (bottom 35% of frame)
- Uses a **PID controller** to calculate how much to steer left or right
- Drives the Mecanum wheels using inverse kinematics to stay centered on the line
- Slows down forward speed when turning sharply

**How to run:**
```bash
python3 5_CamWheel.py
```
Place the robot on the green line and it will start following automatically. Press `Ctrl+C` to stop.

---

### `8_TurnTest.py` — 90° Turn Test
A **standalone turn calibration tool** to test and tune the robot's 90-degree rotation using encoder feedback.

**What it does:**
- Commands the robot to rotate exactly **90 degrees** in place
- Uses wheel encoder tick counting to know when the turn is complete
- Configurable turn direction (`LEFT` or `RIGHT`) and angle gain tuning via `ANGLE_GAIN`

**How to run:**
```bash
python3 8_TurnTest.py
```
The robot will rotate 90° and stop. Adjust `ANGLE_GAIN` if the turn is under/over-rotating.

> ✅ Use this to calibrate turning accuracy before integrating with the full system.

---

### `9_FinalTurn.py` — Line Following + LIDAR-Triggered Avoidance (Integration Ready)
The **most complete camera module** — combines line following with obstacle avoidance triggered by LIDAR (stubbed as keyboard in test mode). This is designed to be merged into the main control file.

**What it does:**
- Follows the green line using camera + PID (same as `5_CamWheel.py`)
- Listens for a LIDAR obstacle trigger on a background thread
- When an obstacle is detected, switches to `MODE_AVOID`: moves sideways → forward → back to lane
- Automatically resumes line following after the maneuver

**Modes:**
| Mode | Behavior |
|---|---|
| `MODE_LINE` | Following the green line normally |
| `MODE_AVOID` | Executing obstacle dodge maneuver |

**How to run (test mode):**
```bash
python3 9_FinalTurn.py
```
- Press `a` to simulate a LIDAR obstacle trigger
- Press `q` to quit

---

## 🔄 Development Progression

```
4_CameraTest.py       ← verify camera sees the line (no movement)
       │
       ▼
5_CamWheel.py         ← robot follows the line
       │
       ▼
8_TurnTest.py         ← calibrate 90° turns
       │
       ▼
9_FinalTurn.py        ← full integration: line follow + obstacle avoid
```

---

## ⚙️ How Line Detection Works

```
Camera Frame
     │
     ▼
Crop bottom 35% (ROI)
     │
     ▼
Convert to HSV color space
     │
     ▼
Mask green pixels (HSV range: [45,80,80] → [75,255,255])
     │
     ▼
Find largest contour → calculate center X
     │
     ▼
Offset = center X - frame center
     │
     ▼
PID controller → steering command → Mecanum wheel drive
```

---

## 🔧 Hardware Used
- Logitech C270 Camera
- Raspberry Pi 5
- Mecanum Wheels (60mm)
- DC Servo Motor GA12 N20
- H-Bridge L298N
- Wheel Encoders (quadrature)

---

## 📌 Notes
- Files `4` and `5` are for **testing only**
- `8_TurnTest.py` is for **calibration only**
- `9_FinalTurn.py` is the **production-ready** version for the full system
- The green line HSV range may need adjustment depending on your lighting conditions — tune `HSV_LOWER` and `HSV_UPPER` in each file