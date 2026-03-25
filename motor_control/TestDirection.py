import lgpio
import numpy as np
import time
import threading

# ================= GPIO CONFIG =================
ENA1, IN1, IN2 = 12, 3, 2
ENB1, IN3, IN4 = 16, 15, 14
ENA2, IN5, IN6 = 18, 20, 21
ENB2, IN8, IN7 = 19, 22, 23

ENC_M1_A, ENC_M1_B = 10, 9
ENC_M2_A, ENC_M2_B = 24, 25
ENC_M3_A, ENC_M3_B = 6, 5
ENC_M4_A, ENC_M4_B = 13, 26

motors = {
    0: {"pwm": ENA1, "inA": IN1, "inB": IN2},  # FR
    1: {"pwm": ENB1, "inA": IN3, "inB": IN4},  # FL
    2: {"pwm": ENA2, "inA": IN5, "inB": IN6},  # BR
    3: {"pwm": ENB2, "inA": IN7, "inB": IN8},  # BL
}

encoders = {
    0: (ENC_M1_A, ENC_M1_B),
    1: (ENC_M2_A, ENC_M2_B),
    2: (ENC_M3_A, ENC_M3_B),
    3: (ENC_M4_A, ENC_M4_B),
}

chip = lgpio.gpiochip_open(4)

for m in motors.values():
    lgpio.gpio_claim_output(chip, m["pwm"])
    lgpio.gpio_claim_output(chip, m["inA"])
    lgpio.gpio_claim_output(chip, m["inB"])

for A, B in encoders.values():
    lgpio.gpio_claim_input(chip, A)
    lgpio.gpio_claim_input(chip, B)

# ================= ENCODER THREAD =================
encoder_count = [0, 0, 0, 0]
lastA = [0, 0, 0, 0]
lock = threading.Lock()
running = True

for i, (A, _) in encoders.items():
    lastA[i] = lgpio.gpio_read(chip, A)

def encoder_loop():
    global running
    while running:
        with lock:
            for i, (A, B) in encoders.items():
                a = lgpio.gpio_read(chip, A)
                if a != lastA[i]:
                    b = lgpio.gpio_read(chip, B)
                    encoder_count[i] += 1 if a == b else -1
                    lastA[i] = a
        time.sleep(0.001)  # 1 kHz

enc_thread = threading.Thread(target=encoder_loop)
enc_thread.start()

# ================= IK / MATH =================
r = 0.03
L = 0.18
l = L / np.sqrt(2)

alpha = np.array([-np.pi/4,  np.pi/4, -3*np.pi/4,  3*np.pi/4])
beta  = np.array([ np.pi/4, -np.pi/4, -np.pi/4,   np.pi/4])
gamma = np.array([ np.pi/4, -np.pi/4, -np.pi/4,   np.pi/4])

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
    def __init__(self, kp, ki, kd, dt, limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt, self.limit = dt, limit
        self.i = 0
        self.prev_e = 0

    def update(self, e):
        self.i += e * self.dt
        d = (e - self.prev_e) / self.dt
        self.prev_e = e
        u = self.kp*e + self.ki*self.i + self.kd*d
        return max(min(u, self.limit), -self.limit)

dt = 0.02
ENC_PPR = 350
MIN_PWM = 6

pid = [
    PID(0.6, 0.0, 0.05, dt, 6),
    PID(0.6, 0.0, 0.05, dt, 6),
    PID(0.6, 0.0, 0.05, dt, 6),
    PID(0.75, 0.0, 0.05, dt, 6),
]

motor_gain = [1.0, 1.0, 1.0, 1.18]

prev_count = [0,0,0,0]

def read_speed(i):
    with lock:
        delta = encoder_count[i] - prev_count[i]
        prev_count[i] = encoder_count[i]
    return (2*np.pi*(delta/ENC_PPR)) / dt

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
        lgpio.gpio_write(chip, m["inA"], 0)
        lgpio.gpio_write(chip, m["inB"], 0)

    lgpio.tx_pwm(chip, m["pwm"], 1000, min(abs(pwm), 100))

# ================= CONTROL HELPERS =================
def reset_control():
    global prev_count
    with lock:
        prev_count = encoder_count.copy()
    for p in pid:
        p.i = 0
        p.prev_e = 0

# ================= MAIN =================
try:
    while True:
        print("nhập x y w (vd: 1 0 0)")
        x, y, w = map(float, input("> ").split())

        reset_control()

        Vg = np.array([[x],[y],[w]])
        phi = (np.linalg.inv(J2) @ J1 @ Vg / 66).flatten()
        if np.max(np.abs(phi)) > 0:
            phi /= np.max(np.abs(phi))

        # ===== START BOOST =====
        t0 = time.time()
        while time.time() - t0 < 0.15:
            for i in range(4):
                set_motor(i, 18 * np.sign(phi[i]))
            time.sleep(0.01)

        current_base = 0

        while True:
            target_base = 20 if abs(y) > 0 else 13
            current_base += np.clip(target_base - current_base,
                                    -80*dt, 80*dt)

            for i in range(4):
                omega = read_speed(i)
                pwm_ff = current_base * phi[i] * motor_gain[i]
                pwm_pid = pid[i].update(-omega)
                pwm = pwm_ff + pwm_pid

                if abs(phi[i]) > 0:
                    pwm = np.sign(phi[i]) * max(abs(pwm), MIN_PWM)

                set_motor(i, pwm)

            time.sleep(dt)

except KeyboardInterrupt:
    running = False
    enc_thread.join()
    for i in range(4):
        set_motor(i, 0)
    lgpio.gpiochip_close(chip)
    print("\nExit cleanly")
