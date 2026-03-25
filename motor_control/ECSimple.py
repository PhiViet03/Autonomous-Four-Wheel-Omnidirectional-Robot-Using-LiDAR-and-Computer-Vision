import lgpio
import time

# ====== CONFIG ======
ENCODERS = {
    0: (10, 9),     # M1 A, B
    1: (25, 24),    # M2
    2: (6, 5),      # M3
    3: (13, 26),    # M4
}

CHIP = 4   # nếu không chạy sẽ thử 0, 1

# ====== INIT ======
chip = lgpio.gpiochip_open(CHIP)

for A, B in ENCODERS.values():
    lgpio.gpio_claim_input(chip, A)
    lgpio.gpio_claim_input(chip, B)

counts = [0, 0, 0, 0]
lastA = [0, 0, 0, 0]

for i, (A, _) in ENCODERS.items():
    lastA[i] = lgpio.gpio_read(chip, A)

print("=== ENCODER TEST ===")
print("Xoay tay bánh xe. Ctrl+C để thoát.\n")

# ====== LOOP ======
try:
    while True:
        for i, (A, B) in ENCODERS.items():
            a = lgpio.gpio_read(chip, A)
            if a != lastA[i]:
                b = lgpio.gpio_read(chip, B)
                if a == b:
                    counts[i] += 1
                else:
                    counts[i] -= 1
                lastA[i] = a

        print(f"ENC = {counts}", end="\r")
        time.sleep(0.001)   # 1 ms polling

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    lgpio.gpiochip_close(chip)
