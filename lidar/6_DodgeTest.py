import lgpio
import numpy as np
import time

# ================= BASIC =================
dt = 0.02

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

# ================= CONTROL =================
WHEEL_MAX_RAD = 8.0
K_FF = 1.1          # giảm nhẹ
PWM_SLEW = 200      # mượt hơn

# ================= PI =================
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

class PID:
    def __init__(self, kp, ki, kd, limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.i = 0.0
        self.prev = 0.0

    def update(self, e, dt):
        self.i += e * dt
        self.i = np.clip(self.i, -self.limit, self.limit)
        d = (e - self.prev) / dt
        self.prev = e
        out = self.kp * e + self.ki * self.i + self.kd * d
        return np.clip(out, -self.limit, self.limit)


wheel_pi = [WheelPI(0.4, 2.0, 60) for _ in range(4)]
heading_pid = PID(kp=2.5, ki=0.0, kd=0.15, limit=0.5)
WHEEL_GAIN_POS = [1.0, 1.0, 1.0, 1.0]   # bánh quay + (CW)
WHEEL_GAIN_NEG = [1.0, 1.0, 1.0, 1.0] # bánh quay - (CCW)


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
    phi_ref = (np.linalg.inv(J2) @ J1 @ Vg).flatten() * WHEEL_MAX_RAD

    for i in range(4):
        if phi_ref[i] >= 0:
            phi_ref[i] *= WHEEL_GAIN_POS[i]
        else:
            phi_ref[i] *= WHEEL_GAIN_NEG[i]


    speed=read_speed()

    for i in range(4):
        pwm=K_FF*phi_ref[i]+wheel_pi[i].update(phi_ref[i]-speed[i])
        step=PWM_SLEW*dt
        pwm=np.clip(pwm,prev_pwm[i]-step,prev_pwm[i]+step)
        prev_pwm[i]=pwm
        set_motor(i,pwm)

# ================= ENCODER ODOM =================
def reset_wheel_pi():
    for pi in wheel_pi:
        pi.i = 0

def reset_move_encoder():
    global enc_last_move
    enc_last_move = enc_count.copy()

def encoder_forward():
    dphi=[]
    for i in range(4):
        d = enc_count[i]-enc_last_move[i]
        dphi.append(2*np.pi*(d/CPR))
    return r * sum(dphi) / 4

def encoder_side():
    dphi=[]
    for i in range(4):
        d = enc_count[i]-enc_last_move[i]
        dphi.append(2*np.pi*(d/CPR))
    return r * (-dphi[0] + dphi[1] + dphi[2] - dphi[3]) / 4

def encoder_rotate():
    dphi=[]
    for i in range(4):
        d = enc_count[i]-enc_last_move[i]
        dphi.append(2*np.pi*(d/CPR))
    return r * (-dphi[0] + dphi[1] - dphi[2] + dphi[3])/(4*l)

J = np.linalg.inv(J2) @ J1
J_pinv = np.linalg.pinv(J)

def encoder_body_delta():
    dphi = []
    for i in range(4):
        d = enc_count[i] - enc_last_move[i]
        dphi.append(2*np.pi*(d/CPR))
    dphi = np.array(dphi).reshape((4,1))

    dV = J_pinv @ dphi
    dx, dy, dtheta = dV.flatten()
    return dx, dy, dtheta

def shutdown_robot():
    for i in range(4):
        set_motor(i, 0)
        lgpio.tx_pwm(chip, motors[i]["pwm"], 0, 0)  # TẮT PWM HẲN
    time.sleep(0.1)
    lgpio.gpiochip_close(chip)

# ================= MOTION =================
def move_straight(dist, speed=0.06):
    reset_move_encoder()
    reset_wheel_pi()

    while True:
        dx, _, _ = encoder_body_delta()
        if abs(dx) >= abs(dist):
            break
        drive(speed*np.sign(dist),0,0)
        time.sleep(dt)

    drive(0,0,0)
    reset_wheel_pi()          # <<< CỰC KỲ QUAN TRỌNG
    time.sleep(0.05)


def move_side(dist, speed=0.06):
    reset_move_encoder()
    reset_wheel_pi()

    while True:
        _, dy, _ = encoder_body_delta()
        if abs(dy) >= abs(dist):
            break
        drive(0, speed*np.sign(dist), 0)
        time.sleep(dt)

    drive(0,0,0)
    reset_wheel_pi()
    time.sleep(0.05)

def rotate(angle):
    reset_move_encoder()
    reset_wheel_pi()

    while True:
        _, _, dth = encoder_body_delta()
        e = angle - dth
        if abs(e) < np.deg2rad(1.0):
            break
        w = heading_pid.update(e, dt)
        drive(0,0,w)
        time.sleep(dt)

    drive(0,0,0)
    reset_wheel_pi()
    time.sleep(0.05)


def stop_and_hold(t):
    t0 = time.time()
    while time.time() - t0 < t:
        drive(0,0,0)
        time.sleep(dt)

# ================= AVOID =================
def avoid_obstacle():
    move_side(0.50)          # sang trái
    stop_and_hold(1)

    move_straight(1)      # đi thẳng ngắn
    stop_and_hold(1)

    move_side(-0.50)         # sang phải
    stop_and_hold(1)

# ================= MAIN =================
try:
    time.sleep(1)
    avoid_obstacle()
    print("=== DONE ===")
    shutdown_robot()
    exit(0)

except KeyboardInterrupt:
    for i in range(4): set_motor(i,0)
    lgpio.gpiochip_close(chip)
