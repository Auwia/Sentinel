from .servo_controller import ServoController

class MotorDriver:
    def __init__(self):
        self.controller = ServoController()

    def move_motor(self, motor_id, position, speed):
        self.controller.set_servo_angle(motor_id, position, speed)

    def cleanup(self):
        self.controller.cleanup()
