import rclpy
from rclpy.node import Node
import subprocess
from scion_types.msg import SubState
from pynput.keyboard import Key, Controller

class RobotLauncher(Node):
    def __init__(self):
        super().__init__('launch_subscriber')
        self.keyboard = Controller()
        self.sub_state_sub = self.create_subscription(SubState, 'submarine_state', self.sub_state_callback, 10)
        self.child = None
        self.vision_chld = None
    def sub_state_callback(self, msg):
        if msg.host_mode == 4:
            self.child = subprocess.Popen(["ros2", "launch", "launch.py"])
        
def main(args=None):
    rclpy.init(args=args)
    sub_state_node = RobotLauncher()
    rclpy.spin(sub_state_node)
    rclpy.shutdown()

