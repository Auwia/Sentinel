import time
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

# Configura il bus I2C
i2c = busio.I2C(SCL, SDA)

# Configura il PCA9685
pca = PCA9685(i2c, address=0x40)  # Usa l'indirizzo 0x40
pca.frequency = 50  # Frequenza PWM (50 Hz per i servo)

# Test: posiziona il servo sul canale 0
try:
    print("Test del servo sul canale 0...")
    # Posizione minima (angolo minimo)
    pca.channels[0].duty_cycle = 0x0666  # Circa 2.5% duty cycle
    print("Angolo minimo impostato (duty 2.5%)")
    time.sleep(2)

    # Posizione centrale (angolo centrale)
    pca.channels[0].duty_cycle = 0x1999  # Circa 7.5% duty cycle
    print("Angolo centrale impostato (duty 7.5%)")
    time.sleep(2)

    # Posizione massima (angolo massimo)
    pca.channels[0].duty_cycle = 0x2CCC  # Circa 12.5% duty cycle
    print("Angolo massimo impostato (duty 12.5%)")
    time.sleep(2)

except KeyboardInterrupt:
    print("Test interrotto manualmente.")
finally:
    print("Spegnimento del canale 0...")
    pca.channels[0].duty_cycle = 0  # Spegni il canale 0
    print("Test completato.")
