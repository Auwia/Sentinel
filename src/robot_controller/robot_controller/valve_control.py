import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import sys

try:
    if sys.platform != "linux" or not hasattr(sys, "real_prefix"):
        raise ImportError("Not running on Raspberry Pi")
    import RPi.GPIO as GPIO
except ImportError:
    class MockGPIO:
        BCM = OUT = HIGH = LOW = None
        def setmode(self, *args): pass
        def setup(self, *args): pass
        def output(self, *args): pass
        def cleanup(self): pass
    GPIO = MockGPIO()

VALVE_PIN = 17  # GPIO pin for valve relay

class ValveControl(Node):
    def __init__(self):
        super().__init__('valve_control')
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(VALVE_PIN, GPIO.OUT)

        self.valve_service = self.create_service(
            ValveControl,
            'valve_control',
            self.handle_valve_control_request
        )

        self.create_subscription(Bool, '/valve', self.valve_callback, 10)
        self.get_logger().info('Valve Control Node Started')

    def valve_callback(self, msg):
        GPIO.output(VALVE_PIN, GPIO.HIGH if msg.data else GPIO.LOW)
        state = "OPEN" if msg.data else "CLOSED"
        self.get_logger().info(f"Valve is now {state}")

    def handle_valve_control_request(self, request, response):
        if request.turn_on:
            GPIO.output(VALVE_PIN, GPIO.HIGH)
            response.success = True
            response.message = "Valve turned ON successfully."
            self.get_logger().info("Valve is now ON.")
        else:
            GPIO.output(VALVE_PIN, GPIO.LOW)
            response.success = True
            response.message = "Valve turned OFF successfully."
            self.get_logger().info("Valve is now OFF.")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ValveControl()
    try:
        rclpy.spin(node)
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()
