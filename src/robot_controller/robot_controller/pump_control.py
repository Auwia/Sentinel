import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# Import RPi.GPIO or use a mock if unavailable
try:
    import RPi.GPIO as GPIO
except ImportError:
    class MockGPIO:
        BCM = OUT = HIGH = LOW = None
        def setmode(self, *args): pass
        def setup(self, *args): pass
        def output(self, *args): pass
        def cleanup(self): pass
    GPIO = MockGPIO()

PUMP_PIN = 18  # GPIO pin for pump relay

class PumpControl(Node):
    def __init__(self):
        super().__init__('pump_control')
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(PUMP_PIN, GPIO.OUT)
        self.create_subscription(Bool, '/pump', self.pump_callback, 10)
        self.get_logger().info('Pump Control Node Started')

    def pump_callback(self, msg):
        GPIO.output(PUMP_PIN, GPIO.HIGH if msg.data else GPIO.LOW)
        state = "ON" if msg.data else "OFF"
        self.get_logger().info(f"Pump is now {state}")

def main(args=None):
    rclpy.init(args=args)
    node = PumpControl()
    try:
        rclpy.spin(node)
    finally:
        GPIO.cleanup()
        rclpy.shutdown()
