import rclpy
from rclpy.node import Node
from custom_interfaces.action import MoveMotors
from rclpy.action import ActionServer

class ServoRelayController(Node):
    def __init__(self):
        super().__init__('servo_relay_control')
        self._action_server = ActionServer(
            self,
            MoveMotors,
            '/servo_relay_control/move_motors',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"Ricevuto goal: Motor ID {goal_handle.request.motor_id}, "
                               f"Speed {goal_handle.request.target_speed}, "
                               f"Angle {goal_handle.request.target_angle}")
        # Implementa la logica per controllare i motori qui
        goal_handle.succeed()
        return MoveMotors.Result(success=True, status_message="Motori spostati con successo!")

def main(args=None):
    rclpy.init(args=args)
    node = ServoRelayController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

