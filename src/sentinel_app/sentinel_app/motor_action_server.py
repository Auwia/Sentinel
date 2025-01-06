import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from custom_interfaces.action import MoveMotors
from adafruit_pca9685 import PCA9685
import busio
from board import SCL, SDA
import json
import os
import time

POSITIONS_FILE = "servo_positions.json"

class MotorActionServer(Node):

    def __init__(self):
        super().__init__('motor_action_server')
        self._action_server = ActionServer(
            self,
            MoveMotors,
            'move_motors',
            self.execute_callback
        )
        self.i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 50
        self.servo_positions = self.load_positions()
        self.get_logger().info("Motor Action Server is running!")

    def load_positions(self):
        if os.path.exists(POSITIONS_FILE):
            with open(POSITIONS_FILE, "r") as file:
                return json.load(file)
        return {str(i): 0 for i in range(16)}

    def save_positions(self):
        with open(POSITIONS_FILE, "w") as file:
            json.dump(self.servo_positions, file)

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f"Executing goal with ID: {goal_handle.goal_id}")
        motor_id = goal_handle.request.motor_id
        target_speed = goal_handle.request.target_speed
        target_angle = goal_handle.request.target_angle

        # Validazione dei parametri
        if not (0 <= motor_id < 16):
            self.get_logger().error(f"Invalid motor_id: {motor_id}. Must be between 0 and 15.")
            goal_handle.abort()
            result = MoveMotors.Result()
            result.success = False
            result.status_message = "Invalid motor_id"
            return result

        if not (0 <= target_angle <= 180):
            self.get_logger().error(f"Invalid target_angle: {target_angle}. Must be between 0 and 180.")
            goal_handle.abort()
            result = MoveMotors.Result()
            result.success = False
            result.status_message = "Invalid target_angle"
            return result

        # Logica per il movimento del motore
        current_angle = self.servo_positions.get(str(motor_id), 0)
        step_delay = 1 / target_speed if target_speed > 0 else 0.1

        if target_angle > current_angle:
            step = 1
        else:
            step = -1

        # Loop per il movimento
        current = current_angle
        while (step > 0 and current <= target_angle) or (step < 0 and current >= target_angle):
            duty_cycle = int(0x0666 + (current / 180.0) * (0x2CCC - 0x0666))
            self.pca.channels[motor_id].duty_cycle = duty_cycle
            self.servo_positions[str(motor_id)] = current
            time.sleep(step_delay)
            current += step

        # Aggiorna la posizione finale
        self.servo_positions[str(motor_id)] = target_angle
        self.save_positions()

        self.get_logger().info(f"Motor {motor_id} reached angle {target_angle}°.")
        goal_handle.succeed()
        result = MoveMotors.Result()
        result.success = True
        result.status_message = "Motor moved successfully"
        return result

    def cleanup(self):
        self.save_positions()
        for channel in range(16):
            self.pca.channels[channel].duty_cycle = 0
        self.pca.deinit()

def main(args=None):
    rclpy.init(args=args)
    node = MotorActionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down server cleanly')
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
