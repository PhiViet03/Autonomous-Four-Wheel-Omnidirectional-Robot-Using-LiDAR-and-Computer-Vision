import cv2
import lgpio
import numpy as np
import time
import sys
import threading

# ================= BASIC =================
dt = 0.02

MODE_LINE  = 0
MODE_AVOID = 1
mode = MODE_LINE

# ================= CAMERA =================
CAM_INDEX = 0
WIDTH, HEIGHT = 1280, 760
ROI_Y_RATIO = 0.65

HSV_LOWER = np.array([45, 80, 80])
HSV_UPPER = np.array([75, 255, 255])
MIN_AREA = 500

# ================= MOTOR GPIO =================
ENA1, IN1, IN2 = 12, 3, 2
ENB1, IN3, IN4 = 16, 15, 14
ENA2, IN5, IN6 = 18, 20, 21
ENB2, IN8, IN7 = 19, 22, 23

motors = {
    0: {"pwm": ENA1, "inA": IN1, "inB": IN2},
    1: {"pwm": ENB1, "inA": IN3, "inB": IN4},
    2: {"pwm": ENA2, "inA": IN5, "inB": IN6},
    3: {"pwm": ENB2, "inA": IN7, "inB": IN8},
}

# ================= ENCODER =================
ENC_PINS = [(10,9),(25,24),(6,5),(13,26)]
CPR = 7 * 50

# ================= ROBOT GEOMETRY =================
r = 0.03
L = 0.18
l = L / np.sqrt(2)

alpha = np.array([ np.pi/4, -np.pi/4,  3*np.pi/4, -3*np.pi/4])
beta  = np.array([ np.pi/4, -np.pi/4, -np.pi/4,  np.pi/4 ])
gamma = np.array([-np.pi/4,  np.pi/4,  np.pi/4, -np.pi/4])

J2 = np.diag([r]*4)
J1 = np.zeros((4,3))
for i in range(4):
    th = alpha[i] + beta[i] + gamma[i]
    J1[i,0] = np.sin(th)
    J1[i,1] = -np.cos(th)
    J1[i,2] = -l * np.cos(beta[i] + gamma[i])
J1 *= 1 / np.cos(np.pi/4)

J = np.linalg.inv(J2) @ J1
J_pinv = np.linalg.pinv(J)

# ================= CONTROL =================
BASE_SPEED = 0.08
MAX_W = 1.6
OFFSET_GAIN = 0.5
LANE_DEADZONE = 0.05

WHEEL_MAX_RAD = 8.0
K_FF = 1.1
PWM_SLEW = 200

# ================= PID =================
class PID:
    def __init__(self, kp, ki, kd, limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.i = 0
        self.prev = 0

    def update(self, e, dt):
        self.i += e * dt
        d = (e - self.prev) / dt
        self.prev = e
        out = self.kp*e + self.ki*self.i + self.kd*d
        return np.clip(out, -self.limit, self.limit)

class WheelPI:
    def __init__(self, kp, ki, limit):
        self.kp = kp
        self.ki = ki
        self.i = 0
        self.limit = limit

    def update(self, e):
        self.i += e * dt
        self.i = np.clip(self.i, -self.limit, self.limit)
        return np.clip(self.kp*e + self.ki*self.i, -self.limit, self.limit)

wheel_pi = [WheelPI(0.4, 2.0, 60) for _ in range(4)]
lane_pid = PID(0.8, 0.0, 0.12, MAX_W)

def reset_wheel_pi():
    for pi in wheel_pi:
        pi.i = 0

# ================= GPIO INIT =================
chip = lgpio.gpiochip_open(4)
for m in motors.values():
    lgpio.gpio_claim_output(chip, m["pwm"])
    lgpio.gpio_claim_output(chip, m["inA"])
    lgpio.gpio_claim_output(chip, m["inB"])

enc_count = [0]*4
enc_last  = [0]*4
enc_last_move = [0]*4

def make_enc_cb(i, b_pin):
    def cb(_, gpio, level, tick):
        b = lgpio.gpio_read(chip, b_pin)
        enc_count[i] += 1 if level == b else -1
    return cb

for i,(a,b) in enumerate(ENC_PINS):
    lgpio.gpio_claim_input(chip,a)
    lgpio.gpio_claim_input(chip,b)
    lgpio.gpio_claim_alert(chip,a,lgpio.BOTH_EDGES)
    lgpio.callback(chip,a,lgpio.BOTH_EDGES,make_enc_cb(i,b))

# ================= MOTOR =================
prev_pwm=[0]*4

def set_motor(i, pwm):
    m = motors[i]
    pwm = int(np.clip(pwm,-100,100))
    if pwm>0:
        lgpio.gpio_write(chip,m["inA"],1)
        lgpio.gpio_write(chip,m["inB"],0)
    elif pwm<0:
        lgpio.gpio_write(chip,m["inA"],0)
        lgpio.gpio_write(chip,m["inB"],1)
    else:
        lgpio.gpio_write(chip,m["inA"],0)
        lgpio.gpio_write(chip,m["inB"],0)
    lgpio.tx_pwm(chip,m["pwm"],1000,abs(pwm))

def read_speed():
    s=[0]*4
    for i in range(4):
        d=enc_count[i]-enc_last[i]
        enc_last[i]=enc_count[i]
        s[i]=2*np.pi*(d/CPR)/dt
    return s

def drive(x,y,w):
    Vg=np.array([[x],[y],[w]])
    phi_ref=(np.linalg.inv(J2)@J1@Vg).flatten()*WHEEL_MAX_RAD
    speed=read_speed()
    for i in range(4):
        pwm=K_FF*phi_ref[i]+wheel_pi[i].update(phi_ref[i]-speed[i])
        step=PWM_SLEW*dt
        pwm=np.clip(pwm,prev_pwm[i]-step,prev_pwm[i]+step)
        prev_pwm[i]=pwm
        set_motor(i,pwm)

# ================= ENCODER ODOM =================
def reset_move_encoder():
    global enc_last_move
    enc_last_move = enc_count.copy()

def encoder_body_delta():
    dphi=[]
    for i in range(4):
        d = enc_count[i]-enc_last_move[i]
        dphi.append(2*np.pi*(d/CPR))
    dphi=np.array(dphi).reshape((4,1))
    dx,dy,dth = (J_pinv @ dphi).flatten()
    return dx,dy,dth

# ================= MOTION =================
def move_straight(dist, speed=0.06):
    reset_move_encoder()
    reset_wheel_pi()
    while abs(encoder_body_delta()[0]) < abs(dist):
        drive(speed*np.sign(dist),0,0)
        time.sleep(dt)
    drive(0,0,0)
    reset_wheel_pi()
    time.sleep(0.05)

def move_side(dist, speed=0.05):
    reset_move_encoder()
    reset_wheel_pi()
    while abs(encoder_body_delta()[1]) < abs(dist):
        drive(0,speed*np.sign(dist),0)
        time.sleep(dt)
    drive(0,0,0)
    reset_wheel_pi()
    time.sleep(0.05)

def avoid_obstacle():
    move_side(-0.5)
    time.sleep(0.5)
    move_straight(1.0)
    time.sleep(0.5)
    move_side(0.5)

# ================= VISION =================
last_lane_x = WIDTH//2

def detect_lane_offset(frame):
    global last_lane_x
    roi=frame[int(HEIGHT*ROI_Y_RATIO):,:]
    hsv=cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,HSV_LOWER,HSV_UPPER)
    cnts,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    if not cnts: return None
    c=max(cnts,key=cv2.contourArea)
    if cv2.contourArea(c)<MIN_AREA: return None
    pts=sorted(c,key=lambda p:p[0][1])[:20]
    x=int(np.mean([p[0][0] for p in pts]))
    last_lane_x=x
    return (x-WIDTH//2)/(WIDTH//2)

# ================= KEYBOARD → LIDAR STUB =================
lidar_trigger = False
exit_flag = False

def keyboard_listener():
    global lidar_trigger, exit_flag
    while True:
        k = sys.stdin.read(1)
        if k == 'a':
            lidar_trigger = True
        elif k == 'q':
            exit_flag = True
            break

threading.Thread(target=keyboard_listener, daemon=True).start()

def lidar_detected():
    global lidar_trigger
    if lidar_trigger:
        lidar_trigger = False
        print("[LIDAR] detected (keyboard)")
        return True
    return False

# ================= MAIN =================
cap=cv2.VideoCapture(CAM_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,HEIGHT)

last_offset=0

try:
    while not exit_flag:

        if mode == MODE_LINE and lidar_detected():
            mode = MODE_AVOID
            continue

        if mode == MODE_AVOID:
            avoid_obstacle()
            reset_wheel_pi()
            mode = MODE_LINE
            continue

        ret,frame=cap.read()
        if not ret: continue
        frame=cv2.flip(frame,1)

        offset=detect_lane_offset(frame)
        if offset is None:
            offset=last_offset*0.9
        else:
            last_offset=offset

        offset*=OFFSET_GAIN
        if abs(offset)<LANE_DEADZONE:
            offset=0

        w=-lane_pid.update(offset,dt)
        x=BASE_SPEED*(1-min(abs(w)/MAX_W,0.6))
        drive(x,0,w)

        time.sleep(dt)

finally:
    for i in range(4):
        lgpio.tx_pwm(chip,motors[i]["pwm"],0,0)
    cap.release()
    lgpio.gpiochip_close(chip)
    print("=== DONE ===")
