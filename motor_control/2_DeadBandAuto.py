import lgpio
import time

# ================= CẤU HÌNH CHÂN (GIỮ NGUYÊN TỪ BƯỚC 1) =================
# Motor 1 (FR) - Chạy Dương
ENA1, IN1, IN2 = 12, 3, 2
ENC_M1_A, ENC_M1_B = 10, 9

# Motor 2 (FL) - Chạy Âm
ENB1, IN3, IN4 = 16, 15, 14
ENC_M2_A, ENC_M2_B = 24, 25

# Motor 3 (BR) - Chạy Dương
ENA2, IN5, IN6 = 18, 20, 21
ENC_M3_A, ENC_M3_B = 6, 5

# Motor 4 (BL) - Chạy Âm
ENB2, IN8, IN7 = 19, 22, 23
ENC_M4_A, ENC_M4_B = 13, 26

# Cấu hình test: Direction 1 = Forward theo Math.py
motors = {
    "M1": {"pwm": ENA1, "inA": IN1, "inB": IN2, "encA": ENC_M1_A, "encB": ENC_M1_B, "dir_sign": 1},
    "M2": {"pwm": ENB1, "inA": IN3, "inB": IN4, "encA": ENC_M2_A, "encB": ENC_M2_B, "dir_sign": -1},
    "M3": {"pwm": ENA2, "inA": IN5, "inB": IN6, "encA": ENC_M3_A, "encB": ENC_M3_B, "dir_sign": 1},
    "M4": {"pwm": ENB2, "inA": IN7, "inB": IN8, "encA": ENC_M4_A, "encB": ENC_M4_B, "dir_sign": -1}
}

chip = lgpio.gpiochip_open(4)

# Setup GPIO
for m in motors.values():
    lgpio.gpio_claim_output(chip, m["pwm"])
    lgpio.gpio_claim_output(chip, m["inA"])
    lgpio.gpio_claim_output(chip, m["inB"])
    lgpio.gpio_claim_input(chip, m["encA"])
    lgpio.gpio_claim_input(chip, m["encB"])

def set_motor(conf, speed_pwm):
    """Hàm điều khiển motor cơ bản"""
    pwm_pin = conf["pwm"]
    # Nhân với dir_sign để đảm bảo luôn test theo chiều TIẾN
    actual_pwm = speed_pwm * conf["dir_sign"] 
    duty = abs(actual_pwm)

    if actual_pwm > 0:
        lgpio.gpio_write(chip, conf["inA"], 1)
        lgpio.gpio_write(chip, conf["inB"], 0)
    elif actual_pwm < 0:
        lgpio.gpio_write(chip, conf["inA"], 0)
        lgpio.gpio_write(chip, conf["inB"], 1)
    else:
        lgpio.gpio_write(chip, conf["inA"], 0)
        lgpio.gpio_write(chip, conf["inB"], 0)
    
    lgpio.tx_pwm(chip, pwm_pin, 1000, duty)

def get_encoder_delta(conf, duration=0.2):
    """Đọc sự thay đổi của encoder trong khoảng thời gian ngắn"""
    pinA = conf["encA"]
    pinB = conf["encB"]
    count = 0
    last_A = lgpio.gpio_read(chip, pinA)
    
    end_time = time.time() + duration
    while time.time() < end_time:
        curr_A = lgpio.gpio_read(chip, pinA)
        curr_B = lgpio.gpio_read(chip, pinB)
        if curr_A != last_A:
            # Chỉ cần đếm xung (absolute), không cần quan tâm chiều ở bước này
            count += 1 
        last_A = curr_A
        time.sleep(0.0005) # Poll nhanh
    return count

try:
    print("=== BƯỚC 2: TÌM VÙNG CHẾT (DEADBAND) ===")
    print("Đang tăng dần PWM để tìm điểm lăn bánh...")
    results = {}

    for name, conf in motors.items():
        print(f"\n--- Đang test {name} ---")
        found_min = False
        
        # Tăng PWM từ 0 đến 60
        for pwm in range(0, 61, 1):
            set_motor(conf, pwm)
            
            # Đợi 0.1s cho motor phản hồi
            time.sleep(0.1)
            
            # Đọc xem có chuyển động không (trong 0.2s)
            delta = get_encoder_delta(conf, duration=0.2)
            
            # Nếu encoder nhảy hơn 5 xung -> Motor đã chạy
            if delta > 5:
                print(f" -> {name} bắt đầu chạy ở PWM = {pwm}")
                results[name] = pwm
                found_min = True
                break
        
        set_motor(conf, 0) # Dừng ngay
        if not found_min:
            print(f" -> {name} KHÔNG CHẠY dù đã lên PWM 60 (Kiểm tra lại nguồn/dây)")
            results[name] = 60 
        time.sleep(1)

    print("\n" + "="*30)
    print("KẾT QUẢ DEADBAND (Min PWM):")
    print("="*30)
    for k, v in results.items():
        print(f"{k}: {v}")
    
    # Gợi ý giá trị Kickstart
    avg_deadband = sum(results.values()) / 4
    suggested_kick = int(avg_deadband * 1.5) + 5
    print("-" * 30)
    print(f"Gợi ý giá trị 'min_startup_pwm' cho PID: {suggested_kick}")
    print("="*30)

except KeyboardInterrupt:
    pass
finally:
    for m in motors.values():
        lgpio.tx_pwm(chip, m["pwm"], 1000, 0)
    lgpio.gpiochip_close(chip)