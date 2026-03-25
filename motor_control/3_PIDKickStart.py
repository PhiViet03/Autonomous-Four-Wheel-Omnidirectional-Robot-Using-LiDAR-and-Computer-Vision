import lgpio
import numpy as np
import time

# ================= GPIO CONFIG =================
ENA1, IN1, IN2 = 12, 3, 2
ENB1, IN3, IN4 = 16, 15, 14
ENA2, IN5, IN6 = 18, 20, 21
ENB2, IN8, IN7 = 19, 22, 23

motors = {
    0: {"pwm": ENA1, "inA": IN1, "inB": IN2},  # FR
    1: {"pwm": ENB1, "inA": IN3, "inB": IN4},  # FL
    2: {"pwm": ENA2, "inA": IN5, "inB": IN6},  # BR
    3: {"pwm": ENB2, "inA": IN7, "inB": IN8},  # BL
}

chip = lgpio.gpiochip_open(4)
for m in motors.values():
    lgpio.gpio_claim_output(chip, m["pwm"])
    lgpio.gpio_claim_output(chip, m["inA"])
    lgpio.gpio_claim_output(chip, m["inB"])

# ================= ROBOT PARAM =================
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

# ================= PID =================
class PID:
    def __init__(self, kp, ki, kd, dt, limit=100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.limit = limit
        self.i = 0
        self.prev_e = 0

    def update(self, e):
        self.i += e * self.dt
        d = (e - self.prev_e) / self.dt
        self.prev_e = e
        out = self.kp * e + self.ki * self.i + self.kd * d
        return np.clip(out, -self.limit, self.limit)

dt = 0.02
pid = [
    PID(0.6, 0.0, 0.05, dt, 20),
    PID(0.6, 0.0, 0.05, dt, 20),
    PID(0.6, 0.0, 0.05, dt, 20),
    PID(0.75,0.0, 0.05, dt, 20),
]

# ================= MOTOR =================
def set_motor(idx, pwm_val):
    m = motors[idx]
    if pwm_val > 0:
        lgpio.gpio_write(chip, m["inA"], 1)
        lgpio.gpio_write(chip, m["inB"], 0)
    elif pwm_val < 0:
        lgpio.gpio_write(chip, m["inA"], 0)
        lgpio.gpio_write(chip, m["inB"], 1)
    else:
        lgpio.gpio_write(chip, m["inA"], 0)
        lgpio.gpio_write(chip, m["inB"], 0)

    lgpio.tx_pwm(chip, m["pwm"], 1000, min(abs(pwm_val), 100))

# ================= HELPER =================
def apply_min_pwm(pwm, pwm_min):
    if pwm == 0:
        return 0
    return np.sign(pwm) * max(abs(pwm), pwm_min)

# ================= DRIVE =================
def drive_from_math(x, y, w, pwm_max=20):
    Vg = np.array([[x], [y], [w]])
    phi_d = np.linalg.inv(J2) @ J1 @ Vg
    phi_d = phi_d.flatten()

    maxv = np.max(np.abs(phi_d))
    if maxv > 0:
        phi_d /= maxv

    # ---- xác định hướng chuyển động ----
    is_rotate  = abs(w) > 0.1 and abs(x) < 0.05 and abs(y) < 0.05
    is_lateral = abs(y) > 0.1 or (abs(x) > 0.1 and abs(y) > 0.1)
    if is_rotate:
        PWM_MIN = 15      # quay cần lực lớn hơn
    elif is_lateral:
        PWM_MIN = 25
    else:
        PWM_MIN = 13

    OMEGA_DEAD = 0.15   # chỉnh từ 0.1 → 0.2 tùy thực tế

    for i in range(4):
        if abs(phi_d[i]) < OMEGA_DEAD:
            set_motor(i, 0)
            continue

        pwm = pid[i].update(phi_d[i])
        pwm *= pwm_max
        pwm = apply_min_pwm(pwm, PWM_MIN)
        set_motor(i, pwm)


    print("omega =", np.round(phi_d, 2), " PWM_MIN =", PWM_MIN)

# ================= MAIN =================
try:
    while True:
        print("nhập x y w (vd: 1 0 0)")
        x, y, w = map(float, input("> ").split())

        while True:
            drive_from_math(x, y, w)
            time.sleep(dt)

except KeyboardInterrupt:
    for i in range(4):
        set_motor(i, 0)
    lgpio.gpiochip_close(chip)
