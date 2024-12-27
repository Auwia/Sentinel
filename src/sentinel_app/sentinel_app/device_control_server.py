import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from sentinel_app.action import ControlDevice  # Import dell'azione definita
import requests

class DeviceControlServer(Node):

    def __init__(self):
        super().__init__('device_control_server')
        self.action_server = ActionServer(
            self,
            ControlDevice,
            'control_device',
            self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Ricevuto comando per controllare dispositivi.')

        # Estrai i dati dalla richiesta
        motor_channels = goal_handle.request.motor_channels
        motor_angles = goal_handle.request.motor_angles
        motor_speeds = goal_handle.request.motor_speeds
        relay_states = goal_handle.request.relay_states

        try:
            # Invia comandi REST per i motori
            for channel, angle, speed in zip(motor_channels, motor_angles, motor_speeds):
                requests.post(
                    'http://192.168.0.178:5000/control',
                    json={
                        'channel': channel,
                        'angle': angle,
                        'speed': speed
                    }
                )

            # Invia comandi REST per i relay
            for i, state in enumerate(relay_states):
                requests.post(
                    f'http://192.168.0.178:5000/relay',
                    json={
                        'relay': i,
                        'state': state
                    }
                )

            # Feedback e completamento
            goal_handle.succeed()
            return ControlDevice.Result(success=True, result_message="Comandi eseguiti con successo.")

        except Exception as e:
            self.get_logger().error(f'Errore durante l\'esecuzione: {e}')
            goal_handle.abort()
            return ControlDevice.Result(success=False, result_message=f'Errore: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DeviceControlServer()
    rclpy.spin(node)
    rclpy.shutdown()
