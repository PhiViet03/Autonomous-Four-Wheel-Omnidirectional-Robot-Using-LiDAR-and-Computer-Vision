import cv2
import lgpio
import numpy as np
import time

# ================= BASIC =================
dt = 0.02

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
ENC_PINS = [(10, 9), (25, 24), (6, 5), (13, 26)]
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

# ================= PARAM =================
BASE_SPEED = 0.08
MAX_W = 1.6

OFFSET_GAIN = 0.5
LANE_DEADZONE = 0.05

WHEEL_MAX_RAD = 8.0
K_FF = 1.2

wheel_pi = [WheelPI(0.4, 2.0, 60) for _ in range(4)]
lane_pid = PID(0.8, 0.0, 0.12, MAX_W)

PWM_SLEW = 250

# ================= GPIO INIT =================
chip = lgpio.gpiochip_open(4)
for m in motors.values():
    lgpio.gpio_claim_output(chip, m["pwm"])
    lgpio.gpio_claim_output(chip, m["inA"])
    lgpio.gpio_claim_output(chip, m["inB"])

enc_count = [0]*4
enc_last  = [0]*4

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

# ================= DRIVE =================
prev_pwm=[0]*4

def drive(x,y,w):
    global prev_pwm
    Vg=np.array([[x],[y],[w]])
    phi_ref=(np.linalg.inv(J2)@J1@Vg).flatten()*WHEEL_MAX_RAD
    speed=read_speed()

    for i in range(4):
        pwm=K_FF*phi_ref[i]+wheel_pi[i].update(phi_ref[i]-speed[i])
        step=PWM_SLEW*dt
        pwm=np.clip(pwm,prev_pwm[i]-step,prev_pwm[i]+step)
        prev_pwm[i]=pwm
        set_motor(i,pwm)

# ================= VISION =================
def detect_lane_offset(frame):
    roi_y=int(HEIGHT*ROI_Y_RATIO)
    roi=frame[roi_y:,:]
    hsv=cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,HSV_LOWER,HSV_UPPER)
    mask=cv2.GaussianBlur(mask,(5,5),0)

    cnts,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    if not cnts: return None
    c=max(cnts,key=cv2.contourArea)
    if cv2.contourArea(c)<MIN_AREA: return None

    pts=sorted(c,key=lambda p:p[0][1])[:20]
    x=int(np.mean([p[0][0] for p in pts]))
    offset=(x-WIDTH//2)/(WIDTH//2)
    return np.clip(offset,-1,1)

# ================= MAIN =================
cap=cv2.VideoCapture(CAM_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,HEIGHT)

last_offset=0

try:
    while True:
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

        BOOST_TH = 0.2    # ngưỡng lệch lớn
        BOOST_GAIN = 1  # độ gắt khi boost

        w_raw = lane_pid.update(offset, dt)

        if abs(offset) > BOOST_TH:
            w = -w_raw * BOOST_GAIN
        else:
            w = -w_raw

        w = np.clip(w, -MAX_W, MAX_W)
        x_cmd=BASE_SPEED*(1-min(abs(w)/MAX_W,0.6))

        drive(x_cmd,0,w)
        time.sleep(dt)

except KeyboardInterrupt:
    for i in range(4): set_motor(i,0)
    cap.release()
    lgpio.gpiochip_close(chip)
