import argparse
import time
import json
import os
from adafruit_pca9685 import PCA9685
import busio
from board import SCL, SDA

POSITIONS_FILE = "servo_positions.json"

class ServoController:
    def __init__(self, i2c_address=0x40, frequency=50):
        """
        Inizializza la scheda PCA9685 e carica le posizioni dei servomotori.
        """
        self.i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(self.i2c, address=i2c_address)
        self.pca.frequency = frequency
        self.servo_positions = self.load_positions()
        print("DEBUG: PCA9685 inizializzata con frequenza 50 Hz.")

    def load_positions(self):
        """
        Carica le posizioni dei servomotori da un file.
        """
        if os.path.exists(POSITIONS_FILE):
            with open(POSITIONS_FILE, "r") as file:
                print("DEBUG: Caricamento posizioni dei servomotori.")
                return json.load(file)
        print("DEBUG: File posizioni non trovato, inizializzazione a 0°.")
        return {str(i): 0 for i in range(16)}  # Default a 0°

    def save_positions(self):
        """
        Salva le posizioni dei servomotori in un file.
        """
        with open(POSITIONS_FILE, "w") as file:
            json.dump(self.servo_positions, file)
        print("DEBUG: Posizioni dei servomotori salvate.")

    def set_servo_angle(self, channel, target_angle, speed):
        """
        Muove il servo a un angolo specifico con una velocità data.

        :param channel: Canale del servo (0-15).
        :param target_angle: Angolo da raggiungere (0-180 gradi).
        :param speed: Velocità del movimento (gradi per secondo).
        """
        if not (0 <= channel < 16):
            print(f"ERROR: Canale {channel} non valido. Deve essere tra 0 e 15.")
            return

        if not (0 <= target_angle <= 180):
            print(f"ERROR: Angolo {target_angle} non valido. Deve essere tra 0 e 180.")
            return

        min_duty = 0x0666  # ~2.5% duty cycle
        max_duty = 0x2CCC  # ~12.5% duty cycle

        current_angle = self.servo_positions.get(str(channel), 0)
        steps = abs(target_angle - current_angle)
        step_delay = 1 / speed  # Tempo tra ogni passo

        if target_angle > current_angle:
            step = 1
        else:
            step = -1

        print(f"DEBUG: Movimento del servo {channel} da {current_angle}° a {target_angle}° a {speed}°/s")
        
        for angle in range(current_angle, target_angle + step, step):
            duty_cycle = int(min_duty + (angle / 180.0) * (max_duty - min_duty))
            self.pca.channels[channel].duty_cycle = duty_cycle
            self.servo_positions[str(channel)] = angle
            time.sleep(step_delay)

        self.servo_positions[str(channel)] = target_angle
        print(f"DEBUG: Servo {channel} raggiunto angolo {target_angle}°")

    def cleanup(self):
        """
        Spegne tutti i servomotori e salva le posizioni.
        """
        print("DEBUG: Pulizia e disattivazione dei servomotori...")
        self.save_positions()
        for channel in range(16):
            self.pca.channels[channel].duty_cycle = 0
        self.pca.deinit()
        print("DEBUG: Pulizia completata.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Controlla un servo con PCA9685")
    parser.add_argument("channel", type=int, help="Canale del servo (0-15)")
    parser.add_argument("angle", type=int, help="Angolo da raggiungere (0-180)")
    parser.add_argument("speed", type=float, help="Velocità del movimento (gradi per secondo)")
    
    args = parser.parse_args()

    try:
        controller = ServoController()
        controller.set_servo_angle(args.channel, args.angle, args.speed)
    except Exception as e:
        print(f"ERROR: {e}")
    finally:
        controller.cleanup()
