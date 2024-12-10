import rclpy
from sentinel_base.servo_controller import ServoController
import time


class MovementManager:
    def __init__(self):
        self.controller = ServoController()

    def move_parallel(self, channels, angles):
        for channel, angle in zip(channels, angles):
            self.controller.set_servo_angle(channel, angle)
        print(f"Movimento parallelo completato: {list(zip(channels, angles))}")

    def stop_all(self, channels):
        for channel in channels:
            self.controller.stop_servo(channel)

    def cleanup(self):
        self.controller.cleanup()


# Test del modulo
if __name__ == "__main__":
    rclpy.init()
    try:
        manager = MovementManager()
        print("Movimento parallelo su canali 0, 1 e 2...")
        manager.move_parallel([0, 1, 2], [0, 45, 90])
        time.sleep(2)

        print("Impostazione angolo 90° per tutti...")
        manager.move_parallel([0, 1, 2], [90, 90, 90])
        time.sleep(2)

        print("Spegni tutti i motori...")
        manager.stop_all([0, 1, 2])

    except KeyboardInterrupt:
        print("Interrotto dall'utente.")

    finally:
        manager.cleanup()
        rclpy.shutdown()
