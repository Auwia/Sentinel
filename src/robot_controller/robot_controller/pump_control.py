import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import os
from .gpio_utils import load_gpio
from custom_interfaces.srv import PumpControl

GPIO = load_gpio()

PUMP_PIN = 27  # GPIO pin for pump relay

class PumpControlNode(Node):
    def __init__(self):
        super().__init__('pump_control')
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(PUMP_PIN, GPIO.OUT, initial=GPIO.HIGH)								   

        # Creazione del servizio
        self.create_service(
            PumpControl,  # Tipo del servizio
            '/pump_control',  # Nome del servizio
            self.handle_pump_control_request  # Callback
        )

        self.get_logger().info('Pump Control Node avviato.')

    def handle_pump_control_request(self, request, response):
        try:
            if request.turn_on:
                GPIO.output(PUMP_PIN, GPIO.LOW)
                response.message = "Pompa accesa con successo."
            else:
                GPIO.output(PUMP_PIN, GPIO.HIGH)
                response.message = "Pompa spenta con successo."
            response.success = True
        except Exception as e:
            response.success = False
            response.message = f"Errore durante il controllo della pompa: {str(e)}"
        self.get_logger().info(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = PumpControlNode()
    try:
        rclpy.spin(node)
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()


