import rclpy
from rclpy.node import Node
from rclpy import spin, shutdown

class CalibrationGUI(Node):
    def __init__(self):
        super().__init__('calibration_gui')
        self.get_logger().info("CalibrationGUI avviato.")

    def run(self):
        self.get_logger().info("CalibrationGUI in esecuzione.")

def main():
    rclpy.init()
    node = CalibrationGUI()
    try:
        spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
