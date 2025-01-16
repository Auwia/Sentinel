import time
from rclpy.node import Node
from rclpy.action import ActionServer

from custom_interfaces.action import MoveMotors

import sys
print(f"Python utilizzato: {sys.executable}")

try:
    from adafruit_pca9685 import PCA9685
    from board import SCL, SDA
    import busio
except ImportError:
    class MockPCA9685:
        class Channel:
            def __init__(self):
                self.duty_cycle = 0

        def __init__(self, *args, **kwargs):
            self.channels = [self.Channel() for _ in range(16)]

        def deinit(self):
            print("Mock PCA9685 deinizializzato.")

    PCA9685 = MockPCA9685
    SCL = SDA = None

    class MockI2C:
        def __init__(self, *args, **kwargs):
            pass

        def __call__(self, *args, **kwargs): 
            return self

    busio = MockI2C()

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_control')

        # Memorizza le posizioni attuali dei servomotori
        self.servo_positions = {str(i): 0 for i in range(16)}

        # PCA9685 initialization
        self.i2c = busio.I2C(SCL, SDA) if SCL and SDA else busio()
        self.pca = PCA9685(self.i2c, address=0x40)
        self.pca.frequency = 50
        self._action_server = ActionServer(
            self,
            MoveMotors,
            '/servo_control/set_servo_angle',
            self.execute_callback
        )
        self.get_logger().info("ServoController avviato e pronto a ricevere comandi.")

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"Ricevuto goal: {goal_handle.request}")
        motor_id = goal_handle.request.motor_id
        target_angle = goal_handle.request.target_angle
        target_speed = goal_handle.request.target_speed
        servo_type = goal_handle.request.servo_type

        try:
            self.set_servo_angle(motor_id, target_angle, target_speed, servo_type)
            goal_handle.succeed()
            return MoveMotors.Result(success=True, status_message="Comando eseguito con successo!")
        except Exception as e:
            goal_handle.abort()
            return MoveMotors.Result(success=False, status_message=f"Errore: {str(e)}")

    def set_servo_angle(self, channel, target_angle=0, speed=0, servo_type=180):
        if not (0 <= channel < 16):
            self.get_logger().error(f"Canale {channel} non valido. Deve essere tra 0 e 15.")
            return

        if servo_type in [180, 270] and not (0 <= target_angle <= servo_type):
            self.get_logger().error(f"Angolo {target_angle} non valido. Deve essere tra 0 e {servo_type}.")
            return

        min_duty = 0x0666
        max_duty = 0x2CCC
        neutral_duty = (min_duty + max_duty) // 2

        if servo_type == 360:
            # Controllo velocità per servomotori a 360°
            if not (-100 <= speed <= 100):
                self.get_logger().error(f"Velocità {speed} non valida. Deve essere tra -100 e 100.")
                return

            if speed > 0:
                duty_cycle = neutral_duty + int((speed / 100.0) * (max_duty - neutral_duty))
            elif speed < 0:
                duty_cycle = neutral_duty + int((speed / 100.0) * (neutral_duty - min_duty))
            else:
                duty_cycle = neutral_duty

            self.pca.channels[channel].duty_cycle = duty_cycle
            self.get_logger().info(f"Servo 360° (Canale {channel}): Velocità: {speed}, Duty cycle: {hex(duty_cycle)}")
        else:
            # Movimento graduale per servomotori standard
            current_angle = self.servo_positions.get(str(channel), 0)
            step = 1 if target_angle > current_angle else -1

            if current_angle == target_angle:
                self.get_logger().info(f"Servo {channel} è già all'angolo {target_angle}°.")
                return

            for angle in range(int(current_angle), int(target_angle) + step, step):
                duty_cycle = int(min_duty + (angle / servo_type) * (max_duty - min_duty))
                self.pca.channels[channel].duty_cycle = duty_cycle
                self.servo_positions[str(channel)] = angle
                time.sleep(1 / speed if speed > 0 else 0)

            # Assicura che raggiunga esattamente l'angolo target
            duty_cycle = int(min_duty + (target_angle / servo_type) * (max_duty - min_duty))
            self.pca.channels[channel].duty_cycle = duty_cycle
            self.servo_positions[str(channel)] = target_angle

            self.get_logger().info(f"Servo {channel} raggiunto angolo {target_angle}°")

    def cleanup(self):
        for channel in range(16):
            self.pca.channels[channel].duty_cycle = 0
        self.pca.deinit()
        self.get_logger().info("Controller PCA9685 pulito e risorse rilasciate.")

def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = ServoController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("ServoController terminato.")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

