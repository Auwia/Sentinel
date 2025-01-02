import time
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

# Configura il bus I2C
i2c = busio.I2C(SCL, SDA)

# Configura il PCA9685
pca = PCA9685(i2c, address=0x40)
pca.frequency = 50

# Funzione per calcolare l'angolo corrispondente al duty cycle
def duty_cycle_to_angle(duty_cycle, duty_min, duty_max):
    angle = (duty_cycle - duty_min) / (duty_max - duty_min) * 360
    return round(angle, 1)

try:
    print("Test del range del servo sul canale 2...")
    print("Osserva il comportamento del motore per determinare il tipo (180°, 270°, 360°, ecc.)")

    # Range tipico di duty cycle per un servo (modifica se necessario)
    duty_min = 0x0666  # Minimo duty cycle (~2.5%)
    duty_max = 0x2400  # Massimo duty cycle (~10%)

    # Ciclo per testare il range
    last_angle = None
    for duty in range(duty_min, duty_max + 1, 0x0080):  # Incrementi di 0x0080
        pca.channels[2].duty_cycle = duty
        angle = duty_cycle_to_angle(duty, duty_min, duty_max)
        print(f"Duty cycle: {hex(duty)}, Angolo calcolato: {angle}°")
        time.sleep(1)  # Attendi per osservare il movimento

        # Rilevamento del limite massimo
        if last_angle is not None and abs(angle - last_angle) < 1:
            print(f"Limite raggiunto intorno a {last_angle}°. Probabile tipo di servo:")
            if last_angle <= 180:
                print(" -> Servo a 180°")
            elif last_angle <= 270:
                print(" -> Servo a 270°")
            else:
                print(" -> Servo a 360°")
            break

        last_angle = angle

    print("Test completato.")

except KeyboardInterrupt:
    print("Test interrotto manualmente.")
finally:
    pca.channels[2].duty_cycle = 0  # Spegni il canale
    print("Canale spento. Test completato.")

import time
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

# Configura il bus I2C
i2c = busio.I2C(SCL, SDA)

# Configura il PCA9685
pca = PCA9685(i2c, address=0x40)
pca.frequency = 50

# Funzione per calcolare l'angolo corrispondente al duty cycle
def duty_cycle_to_angle(duty_cycle, duty_min, duty_max):
    angle = (duty_cycle - duty_min) / (duty_max - duty_min) * 360
    return round(angle, 1)

try:
    print("Test del range del servo sul canale 2...")
    print("Osserva il comportamento del motore per determinare il tipo (180°, 270°, 360°, ecc.)")

    # Range tipico di duty cycle per un servo (modifica se necessario)
    duty_min = 0x0666  # Minimo duty cycle (~2.5%)
    duty_max = 0x2400  # Massimo duty cycle (~10%)

    # Ciclo per testare il range
    last_angle = None
    for duty in range(duty_min, duty_max + 1, 0x0080):  # Incrementi di 0x0080
        pca.channels[2].duty_cycle = duty
        angle = duty_cycle_to_angle(duty, duty_min, duty_max)
        print(f"Duty cycle: {hex(duty)}, Angolo calcolato: {angle}°")
        time.sleep(1)  # Attendi per osservare il movimento

        # Rilevamento del limite massimo
        if last_angle is not None and abs(angle - last_angle) < 1:
            print(f"Limite raggiunto intorno a {last_angle}°. Probabile tipo di servo:")
            if last_angle <= 180:
                print(" -> Servo a 180°")
            elif last_angle <= 270:
                print(" -> Servo a 270°")
            else:
                print(" -> Servo a 360°")
            break

        last_angle = angle

    print("Test completato.")

except KeyboardInterrupt:
    print("Test interrotto manualmente.")
finally:
    pca.channels[2].duty_cycle = 0  # Spegni il canale
    print("Canale spento. Test completato.")

