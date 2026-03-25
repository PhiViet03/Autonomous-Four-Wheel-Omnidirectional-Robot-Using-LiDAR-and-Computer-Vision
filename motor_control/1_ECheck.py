import lgpio
import time
import math

# ================= CẤU HÌNH CHÂN (THEO DỮ LIỆU BẠN GỬI) =================
# ---------- Motor 1 (FR) ----------
ENA1, IN1, IN2 = 12, 3, 2       # PWM=Pin32, DirA=Pin5, DirB=Pin3
ENC_M1_A, ENC_M1_B = 10, 9      # Pin 19, 21

# ---------- Motor 2 (FL) ----------
ENB1, IN3, IN4 = 16, 15, 14     # PWM=Pin36, DirA=Pin10, DirB=Pin8
ENC_M2_A, ENC_M2_B = 25, 24     # Pin 18, 22

# ---------- Motor 3 (BR) ----------
ENA2, IN5, IN6 = 18, 20, 21     # PWM=Pin12, DirA=Pin38, DirB=Pin40
ENC_M3_A, ENC_M3_B = 6, 5       # Pin 31, 29

# ---------- Motor 4 (BL) ----------
# Lưu ý: Code cũ của bạn IN8=22, IN7=23. Giữ nguyên.
ENB2, IN8, IN7 = 19, 22, 23     # PWM=Pin35, DirB=Pin15, DirA=Pin16
ENC_M4_A, ENC_M4_B = 13, 26     # Pin 33, 37

# Mapping
motors = {
    "M1": {"pwm": ENA1, "inA": IN1, "inB": IN2, "encA": ENC_M1_A, "encB": ENC_M1_B, "test_pwm": 30},  # Test Dương
    "M2": {"pwm": ENB1, "inA": IN3, "inB": IN4, "encA": ENC_M2_A, "encB": ENC_M2_B, "test_pwm": -30}, # Test Âm
    "M3": {"pwm": ENA2, "inA": IN5, "inB": IN6, "encA": ENC_M3_A, "encB": ENC_M3_B, "test_pwm": 30},  # Test Dương
    "M4": {"pwm": ENB2, "inA": IN7, "inB": IN8, "encA": ENC_M4_A, "encB": ENC_M4_B, "test_pwm": -30}  # Test Âm
}

# Setup Chip
chip = lgpio.gpiochip_open(4)

# Setup GPIO
for m in motors.values():
    lgpio.gpio_claim_output(chip, m["pwm"])
    lgpio.gpio_claim_output(chip, m["inA"])
    lgpio.gpio_claim_output(chip, m["inB"])
    lgpio.gpio_claim_input(chip, m["encA"])
    lgpio.gpio_claim_input(chip, m["encB"])

# Hàm đọc Encoder đơn giản (chỉ check dấu)
def read_encoder_sign(pinA, pinB, duration=1.0):
    count = 0
    last_A = lgpio.gpio_read(chip, pinA)
    start_t = time.time()
    
    while time.time() - start_t < duration:
        curr_A = lgpio.gpio_read(chip, pinA)
        curr_B = lgpio.gpio_read(chip, pinB)
        if curr_A != last_A:
            if curr_A == curr_B:
                count += 1
            else:
                count -= 1
        last_A = curr_A
        time.sleep(0.001)
    return count

# Hàm điều khiển motor
def run_motor(name, pwm_val):
    cfg = motors[name]
    duty = abs(pwm_val)
    
    # Logic đảo chiều
    if pwm_val > 0:
        lgpio.gpio_write(chip, cfg["inA"], 1)
        lgpio.gpio_write(chip, cfg["inB"], 0)
    else:
        lgpio.gpio_write(chip, cfg["inA"], 0)
        lgpio.gpio_write(chip, cfg["inB"], 1)
        
    lgpio.tx_pwm(chip, cfg["pwm"], 1000, duty)

def stop_motor(name):
    cfg = motors[name]
    lgpio.tx_pwm(chip, cfg["pwm"], 1000, 0)
    lgpio.gpio_write(chip, cfg["inA"], 0)
    lgpio.gpio_write(chip, cfg["inB"], 0)

# ================= MAIN TEST =================
try:
    print("=== BẮT ĐẦU CHECK CHIỀU QUAY (SANITY CHECK) ===")
    print("Mục tiêu: Tất cả bánh phải quay theo chiều TIẾN của xe")
    
    for m_name in ["M1", "M2", "M3", "M4"]:
        target_pwm = motors[m_name]["test_pwm"]
        print(f"\n--- Testing {m_name} với PWM = {target_pwm} ---")
        
        # 1. Chạy motor
        run_motor(m_name, target_pwm)
        
        # 2. Đọc Encoder trong 2 giây
        cnt = read_encoder_sign(motors[m_name]["encA"], motors[m_name]["encB"], duration=2.0)
        
        # 3. Dừng motor
        stop_motor(m_name)
        
        # 4. Đánh giá kết quả
        print(f" -> Encoder Count: {cnt}")
        
        # Kiểm tra LOGIC
        is_spin_ok = False # Bạn nhìn mắt thường để xác nhận
        is_enc_ok = False
        
        # Check dấu Encoder: Phải cùng dấu với PWM cấp vào
        if (target_pwm > 0 and cnt > 0) or (target_pwm < 0 and cnt < 0):
            is_enc_ok = True
            msg = "OK (Cùng dấu)"
        else:
            msg = "SAI (Ngược dấu)"
            
        print(f" -> Check Encoder vs PWM: {msg}")
        print(f" -> [ACTION]: Hãy nhìn bánh xe. Nó có quay TIẾN không?")
        input("    Nhấn Enter để tiếp tục motor tiếp theo...")

except KeyboardInterrupt:
    pass
finally:
    for m in motors: stop_motor(m)
    lgpio.gpiochip_close(chip)
    print("\nĐã tắt GPIO.")