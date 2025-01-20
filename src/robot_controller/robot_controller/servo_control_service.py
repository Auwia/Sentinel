import rclpy
from rclpy.node import Node
from custom_interfaces.srv import SetServoAngle
import time

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


class ServoControlServiceNode(Node):
    def __init__(self):
        super().__init__('servo_control_service')

        self.emergency_flag = False

        # Memorizza le posizioni attuali dei servomotori
        self.servo_positions = {str(i): 0 for i in range(16)}

        # PCA9685 initialization
        self.i2c = busio.I2C(SCL, SDA) if SCL and SDA else busio()
        self.pca = PCA9685(self.i2c, address=0x40)
        self.pca.frequency = 50

        # Creazione del servizio
        self.create_service(
            SetServoAngle,  # Tipo del servizio
            '/servo_control_service',  # Nome del servizio
            self.handle_set_servo_angle_request  # Callback
        )

        self.get_logger().info("ServoControlServiceNode avviato e pronto a ricevere richieste.")

    def handle_set_servo_angle_request(self, request, response):
        motor_id = request.motor_id
        target_angle = request.target_angle
        target_speed = request.target_speed
        servo_type = request.servo_type

        try:
            self.set_servo_angle(motor_id, target_angle, target_speed, servo_type)
            response.success = True
            response.status_message = "Comando eseguito con successo!"
        except Exception as e:
            self.get_logger().error(f"Errore durante il movimento del servo {motor_id}: {str(e)}")
            response.success = False
            response.status_message = f"Errore: {str(e)}"

        return response

    def set_servo_angle(self, channel, target_angle=0, speed=0, servo_type=180):
        if not (0 <= channel < 16):
            self.get_logger().error(f"Canale {channel} non valido. Deve essere tra 0 e 15.")
            raise ValueError(f"Canale {channel} non valido.")

        if servo_type not in [180, 270, 360]:
            self.get_logger().error(f"Tipo di servo {servo_type} non valido. Deve essere 180, 270 o 360.")
            raise ValueError(f"Tipo di servo {servo_type} non valido.")

        if servo_type in [180, 270] and not (0 <= target_angle <= servo_type):
            self.get_logger().error(f"Angolo {target_angle} non valido. Deve essere tra 0 e {servo_type}.")
            raise ValueError(f"Angolo {target_angle} non valido.")

        if speed < 0:
            self.get_logger().error(f"Velocità non valida ({speed}): deve essere maggiore di 0.")
            raise ValueError(f"Velocità {speed} non valida.")

        if speed == 0 or self.emergency_flag:
            self.get_logger().info(f"Velocità impostata a 0. Spegnimento del servo sul canale {channel}.")
            self.pca.channels[channel].duty_cycle = 0  # Spegne il servo
            return

        min_duty = 0x0666
        max_duty = 0x2CCC

        if servo_type == 360:
            # Controllo velocità per servomotori a 360°
            neutral_duty = (min_duty + max_duty) // 2
            duty_cycle = neutral_duty + int((speed / 100.0) * (max_duty - neutral_duty))
            self.pca.channels[channel].duty_cycle = duty_cycle
            self.get_logger().info(f"Servo 360° (Canale {channel}): Velocità: {speed}, Duty cycle: {hex(duty_cycle)}")
        else:
            # Movimento graduale per servomotori standard
            current_angle = self.servo_positions.get(str(channel), 0)
            step = 1 if target_angle > current_angle else -1

            if current_angle == target_angle:
                self.get_logger().info(f"Servo {channel} è già all'angolo {target_angle}°.")
                return

            self.get_logger().info(f"Iniziando movimento del servo {channel} da {current_angle}° a {target_angle}° a velocità {speed}°/s.")
            try:
                for angle in range(int(current_angle), int(target_angle) + step, step):
                    duty_cycle = int(min_duty + (angle / servo_type) * (max_duty - min_duty))
                    self.pca.channels[channel].duty_cycle = duty_cycle
                    self.servo_positions[str(channel)] = angle
                    time.sleep(1 / speed)
            except Exception as e:
                self.get_logger().error(f"Errore durante il movimento del servo {channel}: {e}")
                raise

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
    rclpy.init(args=args)
    node = ServoControlServiceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("ServoControlServiceNode terminato.")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

