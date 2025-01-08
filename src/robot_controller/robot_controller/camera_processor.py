import rclpy
from rclpy.node import Node
from rclpy import spin, shutdown

class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')
        self.get_logger().info("CameraProcessor avviato.")

    def run(self):
        self.get_logger().info("CameraProcessor in esecuzione.")

def main():
    rclpy.init()
    node = CameraProcessor()
    try:
        spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
