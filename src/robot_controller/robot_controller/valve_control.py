import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import os
from .gpio_utils import load_gpio
from custom_interfaces.srv import ValveControl

GPIO = load_gpio()

VALVE_PIN = 17  # GPIO pin for valve relay

class ValveControlNode(Node):
    def __init__(self):
        super().__init__('valve_control')
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(VALVE_PIN, GPIO.OUT, initial=GPIO.HIGH)								   

        # Creazione del servizio
        self.create_service(
            ValveControl,  # Tipo del servizio
            '/valve_control',  # Nome del servizio
            self.handle_valve_control_request  # Callback
        )

        self.get_logger().info('Valve Control Node avviato.')

    def handle_valve_control_request(self, request, response):
        try:
            if request.turn_on:
                GPIO.output(VALVE_PIN, GPIO.HIGH)
                response.message = "Valvola accesa con successo."
            else:
                GPIO.output(VALVE_PIN, GPIO.LOW)
                response.message = "Valvola spenta con successo."
            response.success = True
        except Exception as e:
            response.success = False
            response.message = f"Errore durante il controllo della valvola: {str(e)}"
        self.get_logger().info(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ValveControlNode()
    try:
        rclpy.spin(node)
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()


