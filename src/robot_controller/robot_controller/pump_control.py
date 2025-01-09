import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import os

# Import RPi.GPIO o usa MockGPIO se non disponibile
try:
    import RPi.GPIO as GPIO
    print("RPi.GPIO correttamente caricato.")
except ImportError:
    class MockGPIO:
        BCM = OUT = HIGH = LOW = None
        def setmode(self, *args): pass
        def setup(self, *args): pass
        def output(self, *args): pass
        def cleanup(self): pass
    GPIO = MockGPIO()
    print("Warning: Utilizzando MockGPIO (nessun controllo reale).")

PUMP_PIN = 27  # GPIO pin per il relay della pompa

class PumpControl(Node):
    def __init__(self):
        super().__init__('pump_control')
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(PUMP_PIN, GPIO.OUT)
        self.create_subscription(Bool, '/pump', self.pump_callback, 10)
        self.get_logger().info('Pump Control Node avviato.')

    def pump_callback(self, msg):
        GPIO.output(PUMP_PIN, GPIO.HIGH if msg.data else GPIO.LOW)
        state = "ON" if msg.data else "OFF"
        self.get_logger().info(f"La pompa è ora {state}.")

def main(args=None):
    rclpy.init(args=args)
    node = PumpControl()
    try:
        rclpy.spin(node)
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

