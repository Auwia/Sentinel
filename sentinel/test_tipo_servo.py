import time
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

# Configura il bus I2C
i2c = busio.I2C(SCL, SDA)

# Configura il PCA9685
pca = PCA9685(i2c, address=0x40)  # Usa l'indirizzo 0x40
pca.frequency = 50  # Frequenza PWM (50 Hz per i servo)

# Test del motore
try:
    print("Test del servo sul canale 0...")

    # Range tipico di duty cycle per un servo (modifica se necessario):
    duty_min = 0x0666  # Circa 2.5% duty cycle (angolo minimo o rotazione lenta CCW)
    duty_center = 0x1999  # Circa 7.5% duty cycle (centro o stop per rotazione continua)
    #duty_max = 0x2CCC  # Circa 12.5% duty cycle (angolo massimo o rotazione veloce CW)
    duty_max = 0x2400

    print("1. Movimento al minimo (2.5% duty cycle)...")
    pca.channels[0].duty_cycle = duty_min
    print("Osserva il comportamento: si muove verso il minimo o inizia a ruotare lentamente CCW?")
    time.sleep(3)

    print("2. Posizione centrale (7.5% duty cycle)...")
    pca.channels[0].duty_cycle = duty_center
    print("Osserva il comportamento: si ferma o si sposta verso il centro?")
    time.sleep(3)

    print("3. Movimento al massimo (12.5% duty cycle)...")
    pca.channels[0].duty_cycle = duty_max
    print("Osserva il comportamento: si muove verso il massimo o ruota rapidamente CW?")
    time.sleep(3)

    print("Test completato. Torno alla posizione centrale...")
    pca.channels[0].duty_cycle = duty_center
    time.sleep(2)

except KeyboardInterrupt:
    print("Test interrotto manualmente.")
finally:
    print("Spegnimento del canale 0...")
    pca.channels[0].duty_cycle = 0  # Spegni il canale 0
    print("Test completato.")
