import time
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

# Configura il bus I2C
i2c = busio.I2C(SCL, SDA)

# Configura il PCA9685
pca = PCA9685(i2c, address=0x40)  # Usa l'indirizzo 0x40
pca.frequency = 60  # Frequenza PWM (60 Hz per i servo)

# Test: modifica il duty cycle sul canale 1
try:
    print("Test PCA9685: accensione PWM sul canale 1...")
    while True:
        for duty in range(0, 0xFFFF, 0x0FFF):  # Incremento graduale del duty cycle
            pca.channels[1].duty_cycle = duty  # Canale 1
            time.sleep(0.1)
        print("Inversione del duty cycle...")
        for duty in range(0xFFFF, 0, -0x0FFF):
            pca.channels[1].duty_cycle = duty  # Canale 1
            time.sleep(0.1)
except KeyboardInterrupt:
    print("Test interrotto manualmente.")
finally:
    print("Spegnimento del canale 1...")
    pca.channels[1].duty_cycle = 0  # Spegni il canale 1
    print("Test completato.")
