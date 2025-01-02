import time
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

# Configurazione I2C
i2c = busio.I2C(SCL, SDA)

# Configurazione PCA9685
pca = PCA9685(i2c)
pca.frequency = 60  # Frequenza per i servomotori

# Test: Fai oscillare il segnale PWM su un canale (esempio: canale 0)
channel = pca.channels[0]  # Canale 0
try:
    print("Test PCA9685 avviato!")
    while True:
        print("Accensione PWM...")
        channel.duty_cycle = 0x7FFF  # 50% duty cycle
        time.sleep(1)
        print("Spegnimento PWM...")
        channel.duty_cycle = 0x0000  # Spegni
        time.sleep(1)
except KeyboardInterrupt:
    print("Test interrotto manualmente.")
finally:
    print("Reset dei canali...")
    for i in range(16):
        pca.channels[i].duty_cycle = 0  # Spegni tutti i canali
    print("Test completato!")
