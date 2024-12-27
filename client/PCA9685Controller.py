import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from adafruit_pca9685 import PCA9685
import busio
from board import SCL, SDA

class PCA9685Controller(Node):
    def __init__(self):
        super().__init__('pca9685_controller')
        
        # Setup PCA9685
        self.i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 50
        self.get_logger().info('PCA9685 initialized')

        # Subscriber for control commands
        self.subscription = self.create_subscription(
            String,
            '/pca9685/control',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        # Parse message: Example "CH:0,ANGLE:45"
        try:
            data = dict(item.split(':') for item in msg.data.split(','))
            channel = int(data.get('CH', -1))
            angle = int(data.get('ANGLE', 0))

            if 0 <= channel < 16 and 0 <= angle <= 180:
                self.set_servo_angle(channel, angle)
                self.get_logger().info(f'Set channel {channel} to angle {angle}°')
            else:
                self.get_logger().error('Invalid channel or angle range')

        except Exception as e:
            self.get_logger().error(f'Failed to parse command: {str(e)}')

    def set_servo_angle(self, channel, angle):
        min_duty = 0x0666  # ~2.5% duty cycle
        max_duty = 0x2CCC  # ~12.5% duty cycle
        duty_cycle = int(min_duty + (angle / 180.0) * (max_duty - min_duty))
        self.pca.channels[channel].duty_cycle = duty_cycle

def main(args=None):
    rclpy.init(args=args)
    node = PCA9685Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
