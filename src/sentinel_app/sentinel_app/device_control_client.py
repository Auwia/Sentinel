import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sentinel_app.action import ControlDevice

class DeviceControlClient(Node):

    def __init__(self):
        super().__init__('device_control_client')
        self.action_client = ActionClient(self, ControlDevice, 'control_device')

    def send_goal(self, motor_channels, motor_angles, motor_speeds, relay_states):
        goal_msg = ControlDevice.Goal()
        goal_msg.motor_channels = motor_channels
        goal_msg.motor_angles = motor_angles
        goal_msg.motor_speeds = motor_speeds
        goal_msg.relay_states = relay_states

        self.get_logger().info('Inviando comando ai dispositivi...')
        self.action_client.wait_for_server()
        self.action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DeviceControlClient()
    node.send_goal([0, 1, 2], [90.0, 45.0, 180.0], [30.0, 30.0, 30.0], [True, False])
    rclpy.spin(node)
    rclpy.shutdown()
