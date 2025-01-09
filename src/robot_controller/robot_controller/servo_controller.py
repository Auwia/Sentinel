import time
from rclpy.node import Node
from custom_interfaces.action import MoveMotors
from rclpy.action import ActionServer

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

    busio = MockI2C


class ServoRelayController(Node):
    def __init__(self):
        super().__init__('servo_relay_control')
        self.i2c = busio.I2C(SCL, SDA) if SCL and SDA else busio()
        self.pca = PCA9685(self.i2c, address=0x40)
        self.pca.frequency = 50
        self._action_server = ActionServer(
            self,
            MoveMotors,
            '/servo_relay_control/set_servo_angle',
            self.execute_callback
        )
        self.get_logger().info("ServoRelayController avviato e pronto a ricevere comandi.")

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
        min_duty = 0x0666
        max_duty = 0x2CCC
        neutral_duty = (min_duty + max_duty) // 2

        if servo_type == 360:
            if speed > 0:
                duty_cycle = neutral_duty + int((speed / 100.0) * (max_duty - neutral_duty))
            elif speed < 0:
                duty_cycle = neutral_duty + int((speed / 100.0) * (neutral_duty - min_duty))
            else:
                duty_cycle = neutral_duty
        else:
            angle_ratio = target_angle / servo_type
            duty_cycle = int(min_duty + angle_ratio * (max_duty - min_duty))

        self.pca.channels[channel].duty_cycle = duty_cycle
        action_type = "Speed" if servo_type == 360 else "Angle"
        action_value = speed if servo_type == 360 else target_angle
        self.get_logger().info(f"Servo (Type {servo_type}°): Canale {channel}, {action_type}: {action_value}, Duty cycle: {hex(duty_cycle)}")

    def cleanup(self):
        for channel in range(16):
            self.pca.channels[channel].duty_cycle = 0
        self.pca.deinit()
        self.get_logger().info("Controller PCA9685 pulito e risorse rilasciate.")


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = ServoRelayController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("ServoRelayController terminato.")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

