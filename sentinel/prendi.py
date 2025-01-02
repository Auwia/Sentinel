from adafruit_pca9685 import PCA9685
import busio
from board import SCL, SDA
import time

def test_servo(pca, channel, duty_cycle):
    """
    Imposta un duty cycle per testare il servo.
    """
    print(f"DEBUG: Impostazione del duty cycle {hex(duty_cycle)} sul canale {channel}")
    pca.channels[channel].duty_cycle = duty_cycle
    time.sleep(2)

if __name__ == '__main__':
    try:
        # Configurazione della PCA9685
        print("DEBUG: Inizializzazione del bus I2C...")
        i2c = busio.I2C(SCL, SDA)
        print("DEBUG: Bus I2C inizializzato.")

        print("DEBUG: Inizializzazione della PCA9685...")
        pca = PCA9685(i2c)
        pca.frequency = 50
        print("DEBUG: PCA9685 inizializzata con frequenza 50 Hz.")

        # Test del servo 0
        print("DEBUG: Test del servo 0 con diversi duty cycle...")
        test_servo(pca, 0, 0x0666)  # Angolo minimo

        # Test del servo 2
        print("DEBUG: Test del servo 2 con duty cycle 0x1866...")
        test_servo(pca, 2, 0x1866)  # Valore specificato

    except Exception as e:
        print(f"ERROR: Si è verificato un errore: {e}")

    finally:
        # Disattiva i servomotori e libera le risorse
        print("DEBUG: Pulizia e disattivazione dei servomotori...")
        for channel in range(16):
            pca.channels[channel].duty_cycle = 0
        pca.deinit()
        print("DEBUG: Pulizia completata.")

