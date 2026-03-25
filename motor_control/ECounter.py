import lgpio
import time
import math
import threading
import csv

# ---------- Motor 1 ----------
ENA1 = 12   # PWM pin      -> (Pin 32)
IN1  = 3    # Dir A        -> (Pin 5)
IN2  = 2    # Dir B        -> (Pin 3)

# ---------- Motor 2 ----------
ENB1 = 16   # PWM pin      -> (Pin 36)
IN3  = 15   # Dir A        -> (Pin 10)
IN4  = 14   # Dir B        -> (Pin 8)

# ---------- Motor 3 ----------
ENA2 = 18   # PWM pin      -> (Pin 12)
IN5  = 20   # Dir A        -> (Pin 38)
IN6  = 21   # Dir B        -> (Pin 40)

# ---------- Motor 4 ----------
ENB2 = 19   # PWM pin      -> (Pin 35)
IN8  = 22   # Dir B        -> (Pin 15)
IN7  = 23   # Dir A        -> (Pin 16)

# ---------- Encoder pins ----------
ENC_M1_A, ENC_M1_B = 10, 9     # M1 -> (Pin 19, Pin 21)
ENC_M2_A, ENC_M2_B = 24, 25    # M2 -> (Pin 18, Pin 22)
ENC_M3_A, ENC_M3_B = 6, 5      # M3 -> (Pin 31, Pin 29)
ENC_M4_A, ENC_M4_B = 13, 26    # M4 -> (Pin 33, Pin 37)

# ---------- Wheel parameters ----------
WHEEL_RADIUS = 0.03          # 30 mm
PULSES_PER_REV = 350         # 7 PPR * 50 gear ratio
CIRCUMFERENCE = 2 * math.pi * WHEEL_RADIUS

# ============== Setup ============== #
chip = lgpio.gpiochip_open(4)

# Motor pins setup
for pin in [IN1, IN2, ENA1, IN3, IN4, ENB1, IN5, IN6, ENA2, IN7, IN8, ENB2]:
    lgpio.gpio_claim_output(chip, pin)

# Encoder pins setup
for pin in [ENC_M1_A, ENC_M1_B, ENC_M2_A, ENC_M2_B,
            ENC_M3_A, ENC_M3_B, ENC_M4_A, ENC_M4_B]:
    lgpio.gpio_claim_input(chip, pin)

# Encoder counters
encoder_counts = {"M1": 0, "M2": 0, "M3": 0, "M4": 0}

# Polling thread flag
running = True

# ============== Encoder Polling Thread ============== #
def encoder_poll():
    global encoder_counts

    lastA = {
        "M1": lgpio.gpio_read(chip, ENC_M1_A),
        "M2": lgpio.gpio_read(chip, ENC_M2_A),
        "M3": lgpio.gpio_read(chip, ENC_M3_A),
        "M4": lgpio.gpio_read(chip, ENC_M4_A),
    }

    while running:
        # Loop over all motors
        encoders = {
            "M1": (ENC_M1_A, ENC_M1_B),
            "M2": (ENC_M2_A, ENC_M2_B),
            "M3": (ENC_M3_A, ENC_M3_B),
            "M4": (ENC_M4_A, ENC_M4_B)
        }
        for m, (a, b) in encoders.items():
            stateA = lgpio.gpio_read(chip, a)
            stateB = lgpio.gpio_read(chip, b)
            if stateA != lastA[m]:
                encoder_counts[m] += 1 if stateA == stateB else -1
                lastA[m] = stateA
        time.sleep(0.001)  # 1 ms polling

# Start encoder thread
thread = threading.Thread(target=encoder_poll, daemon=True)
thread.start()

# ============== Motor Control Functions ============== #
def set_motor(motor, speed):
    """Set motor speed (-100..100%)"""
    duty = min(abs(speed), 100)

    motor_pins = {
        "M1": (IN1, IN2, ENA1),
        "M2": (IN3, IN4, ENB1),
        "M3": (IN5, IN6, ENA2),
        "M4": (IN7, IN8, ENB2),
    }

    if motor not in motor_pins:
        return

    inA, inB, pwm = motor_pins[motor]

    if speed > 0:
        lgpio.gpio_write(chip, inA, 1)
        lgpio.gpio_write(chip, inB, 0)
    elif speed < 0:
        lgpio.gpio_write(chip, inA, 0)
        lgpio.gpio_write(chip, inB, 1)
    else:
        lgpio.gpio_write(chip, inA, 0)
        lgpio.gpio_write(chip, inB, 0)
        duty = 0

    lgpio.tx_pwm(chip, pwm, 1000, duty)  # 1 kHz PWM

def stop_all():
    """Stop all motors"""
    for m in ["M1", "M2", "M3", "M4"]:
        set_motor(m, 0)

def get_speed(motor, dt):
    """Compute speed (m/s) from encoder counts in interval dt"""
    counts = encoder_counts[motor]
    encoder_counts[motor] = 0
    revs = counts / PULSES_PER_REV
    distance = revs * CIRCUMFERENCE
    return distance / dt

# ============== Test + Logging ============== #
def test_motor(motor, pwm_val=50, duration=10, logfile="motor_log.csv"):
    """Run one motor and log speed vs time for tau/K estimation"""
    encoder_counts[motor] = 0
    set_motor(motor, 0)
    time.sleep(1)  # ensure motor stopped

    direction = "FWD" if pwm_val > 0 else "REV"
    print(f"[INFO] Starting {motor} test {direction} at {abs(pwm_val)}% for {duration}s...")

    set_motor(motor, pwm_val)

    start_time = time.time()
    t_prev = start_time

    with open(logfile, "a", newline="") as f:
        writer = csv.writer(f)
        while time.time() - start_time < duration:
            t_now = time.time()
            dt = t_now - t_prev
            speed = get_speed(motor, dt)
            writer.writerow([t_now - start_time, motor, pwm_val, direction, speed])
            t_prev = t_now
            time.sleep(0.1)

    set_motor(motor, 0)
    print(f"[DONE] {motor} {direction} test completed.\n")

# ============== Main Script ============== #
wheel_mask = {
    "M1": True,
    "M2": True,
    "M3": True,
    "M4": True
}

try:
    print("=== Starting Selective Motor Tau/K Logging ===")
    with open("motor_log.csv", "w", newline="") as f:
        csv.writer(f).writerow(["time", "motor", "pwm", "direction", "speed"])  # header

    for m in ["M1", "M2", "M3", "M4"]:
        # Kiểm tra xem motor này có được phép chạy không
        if wheel_mask.get(m, False): 
            # Forward test
            test_motor(m, pwm_val=10, duration=5)
            time.sleep(2)
            # Reverse test
            test_motor(m, pwm_val=-10, duration=5)
            time.sleep(2)
        else:
            print(f"[SKIP] Motor {m} is disabled in wheel_mask.")

    stop_all()
    print("\nAll enabled motor tests completed. Data saved to motor_log.csv")

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    running = False
    thread.join(timeout=1)
    stop_all()
    lgpio.gpiochip_close(chip)
    print("GPIO closed, program ended cleanly.")
