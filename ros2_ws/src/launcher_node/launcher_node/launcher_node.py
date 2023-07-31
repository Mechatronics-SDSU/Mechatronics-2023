import rclpy
from rclpy.node import Node
import subprocess
from scion_types.msg import SubState

class RobotLauncher(Node):
    def __init__(self):
        super().__init__('launch_subscriber')
        self.sub_state_sub = self.create_subscription(SubState, 'submarine_state', self.sub_state_callback, 10)
        self.child = None
    def sub_state_callback(self, msg):
        if msg.host_mode == 4:
            self.child = subprocess.Popen(["ros2", "launch", "launch.py"])
        if msg.host_mode == 0:
            self.child.terminate()

def main(args=None):
    rclpy.init(args=args)
    sub_state_node = RobotLauncher()
    rclpy.spin(sub_state_node)
    rclpy.shutdown()

