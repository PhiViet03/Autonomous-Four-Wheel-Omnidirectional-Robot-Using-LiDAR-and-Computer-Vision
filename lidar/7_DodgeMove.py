
# ================= FULL MERGED CODE (FIXED) =================

import sys          #Test né
import select
import termios
import tty

import lgpio
import numpy as np
import time
import cv2

# ================= CAMERA =================
WIDTH, HEIGHT = 640, 480
ROI_Y_RATIO = 0.6

HSV_LOWER = np.array([45, 80, 80])
HSV_UPPER = np.array([75, 255, 255])
MIN_AREA = 2000

OFFSET_DEAD = 15
OFFSET_ALPHA = 0.2

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

kernel = np.ones((5, 5), np.uint8)
offset_filt = 0
delta_filt = 0

# ================= GPIO =================
ENA1, IN1, IN2 = 12, 3, 2    # FR
ENB1, IN3, IN4 = 16, 15, 14 # FL
ENA2, IN5, IN6 = 18, 20, 21 # BR
ENB2, IN8, IN7 = 19, 22, 23 # BL

motors = {
    0: {"pwm": ENA1, "inA": IN1, "inB": IN2},
    1: {"pwm": ENB1, "inA": IN3, "inB": IN4},
    2: {"pwm": ENA2, "inA": IN5, "inB": IN6},
    3: {"pwm": ENB2, "inA": IN7, "inB": IN8},
}

# ================= ENCODER =================
ENCODERS = {
    0: (10, 9),
    1: (25, 24),
    2: (6, 5),
    3: (13, 26),
}

enc_count = [0, 0, 0, 0]
last_state = {}

chip = lgpio.gpiochip_open(4)

for m in motors.values():
    lgpio.gpio_claim_output(chip, m["pwm"])
    lgpio.gpio_claim_output(chip, m["inA"])
    lgpio.gpio_claim_output(chip, m["inB"])

for i, (A, B) in ENCODERS.items():
    lgpio.gpio_claim_input(chip, A)
    lgpio.gpio_claim_input(chip, B)
    last_state[i] = (lgpio.gpio_read(chip, A),
                     lgpio.gpio_read(chip, B))

# ================= MOTOR =================
def set_motor(i, pwm):
    m = motors[i]
    if pwm > 0:
        lgpio.gpio_write(chip, m["inA"], 1)
        lgpio.gpio_write(chip, m["inB"], 0)
    elif pwm < 0:
        lgpio.gpio_write(chip, m["inA"], 0)
        lgpio.gpio_write(chip, m["inB"], 1)
    else:
        lgpio.gpio_write(chip, m["inA"], 1)
        lgpio.gpio_write(chip, m["inB"], 1)
        lgpio.tx_pwm(chip, m["pwm"], 1000, 0)
        return
    lgpio.tx_pwm(chip, m["pwm"], 1000, min(abs(pwm), 100))

def stop_all():
    for i in range(4):
        set_motor(i, 0)

# ================= ENCODER UTILS =================
def update_encoder(i):
    A, B = ENCODERS[i]
    a = lgpio.gpio_read(chip, A)
    b = lgpio.gpio_read(chip, B)
    la, lb = last_state[i]

    if (a, b) != (la, lb):
        if la == a:
            enc_count[i] += 1 if a != b else -1
        else:
            enc_count[i] += -1 if a != b else 1
    last_state[i] = (a, b)

prev_enc = enc_count.copy()
prev_t = time.time()

def get_wheel_speed():
    global prev_enc, prev_t
    for i in range(4):
        update_encoder(i)

    t = time.time()
    dt = max(t - prev_t, 1e-3)
    speeds = []
    for i in range(4):
        speeds.append((enc_count[i] - prev_enc[i]) / dt)
    prev_enc = enc_count.copy()
    prev_t = t
    return speeds

def stop_with_encoder(t=0.25):
    t0 = time.time()
    while time.time() - t0 < t:
        speeds = get_wheel_speed()
        for i in range(4):
            if abs(speeds[i]) > 1:
                set_motor(i, -int(0.3 * speeds[i]))
            else:
                set_motor(i, 0)
        time.sleep(0.02)
    stop_all()

# ================= IK =================
r = 0.03
L = 0.18
l = L / np.sqrt(2)

alpha = np.array([ np.pi/4, -np.pi/4,  3*np.pi/4, -3*np.pi/4])
beta  = np.array([ np.pi/4, -np.pi/4, -np.pi/4,  np.pi/4 ])
gamma = np.array([-np.pi/4,  np.pi/4,  np.pi/4, -np.pi/4])

J2 = np.diag([r, r, r, r])
J1 = np.zeros((4, 3))
for i in range(4):
    th = alpha[i] + beta[i] + gamma[i]
    J1[i, 0] = np.sin(th)
    J1[i, 1] = -np.cos(th)
    J1[i, 2] = -l * np.cos(beta[i] + gamma[i])
J1 *= 1 / np.cos(np.pi / 4)

# ================= LINE PID =================
class PID:
    def __init__(self, kp, ki, kd, dt, limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt, self.limit = dt, limit
        self.i = 0
        self.prev = 0
    def update(self, e):
        self.i += e * self.dt
        d = (e - self.prev) / self.dt
        self.prev = e
        return np.clip(self.kp*e + self.ki*self.i + self.kd*d,
                       -self.limit, self.limit)

dt = 0.02
pid_line = PID(0.01, 0, 0.002, dt, 1)

# ================= WHEEL TO PWM (FROM CODE 1) =================
BASE_PWM = 13
DELTA_PWM = 5
RIGHT_WHEELS = [0, 2]
LEFT_WHEELS  = [1, 3]

def wheel_to_pwm(i, phi, offset, delta):
    sign = 1 if phi >= 0 else -1

    if abs(offset) < OFFSET_DEAD:
        pwm = BASE_PWM
    else:
        if offset > 0:
            pwm = BASE_PWM + delta if i in LEFT_WHEELS else BASE_PWM - (DELTA_PWM - 2)
        else:
            pwm = BASE_PWM + delta if i in RIGHT_WHEELS else BASE_PWM - (DELTA_PWM - 2)

    if i == 3:
        pwm += 1

    return sign * int(pwm)

# ================= DRIVE FROM MATH (FIXED) =================
def drive_from_math(x, w):
    Vg = np.array([[x], [0], [w]])
    phi_ref = (np.linalg.inv(J2) @ J1 @ Vg).flatten()

    m = np.max(np.abs(phi_ref))
    if m > 1:
        phi_ref /= m

    for i in range(4):
        pwm = wheel_to_pwm(i, phi_ref[i], offset_filt, delta_filt)
        set_motor(i, pwm)

# ================= AVOID LOGIC =================
def drive_ik_closed(x, y, w, base_pwm):
    Vg = np.array([[x], [y], [w]])
    phi_cmd = (np.linalg.inv(J2) @ J1 @ Vg).flatten()
    m = np.max(np.abs(phi_cmd))
    if m > 1:
        phi_cmd /= m

    speeds = get_wheel_speed()
    for i in range(4):
        error = phi_cmd[i] - speeds[i] * 0.01
        pwm = int(base_pwm * (phi_cmd[i] + 0.1 * error))
        set_motor(i, pwm)

def avoid_obstacle():
    t0 = time.time()
    while time.time() - t0 < 0.8:
        drive_ik_closed(0.8, 0.8, 0.0, base_pwm=30)
        time.sleep(0.02)
    stop_with_encoder()
    time.sleep(0.3)
    t0 = time.time()
    while time.time() - t0 < 0.8:
        drive_ik_closed(0.8, -0.8, 0.0, base_pwm=30)
        time.sleep(0.02)
    stop_with_encoder()

# ================= VISION =================
def find_centroid(mask):
    cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts: return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < MIN_AREA: return None
    M = cv2.moments(c)
    return int(M["m10"]/M["m00"]) if M["m00"] else None

# ================= MAIN =================
AVOIDING = False

def obstacle_detected():          #Test né
    key = read_key()
    if key == 'a':
        print(">>> TEST AVOID TRIGGERED")
        return True
    elif key == 'q':
        raise KeyboardInterrupt
    return False

fd = sys.stdin.fileno()          #Test né
old_settings = termios.tcgetattr(fd)
tty.setcbreak(fd)

def read_key():          #Test né
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr:
        return sys.stdin.read(1)
    return None

try:
    while True:
        if obstacle_detected() and not AVOIDING:
            AVOIDING = True
            avoid_obstacle()
            AVOIDING = False
            continue

        ret, frame = cap.read()
        if not ret:
            continue

        roi = frame[int(HEIGHT*ROI_Y_RATIO):, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        cx = find_centroid(mask)
        if cx is not None:
            offset = cx - WIDTH//2
            if abs(offset) < OFFSET_DEAD:
                offset = 0
            offset_filt = (1-OFFSET_ALPHA)*offset_filt + OFFSET_ALPHA*offset
        else:
            offset_filt *= 0.1

        w = pid_line.update(offset_filt)
        delta_target = DELTA_PWM * min(abs(w) / 0.6, 1.0)
        delta_filt = 0.8 * delta_filt + 0.2 * delta_target

        drive_from_math(1, w)
        time.sleep(dt)

except KeyboardInterrupt:
    stop_all()
    cap.release()
    lgpio.gpiochip_close(chip)
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)          #Test né
