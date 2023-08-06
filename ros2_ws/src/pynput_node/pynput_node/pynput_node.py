import rclpy
from rclpy.node import Node
import subprocess
from scion_types.msg import SubState
from pynput.keyboard import Key, Controller
from time import sleep

class Pynput(Node):
    def __init__(self):
        super().__init__('pynput_subscriber')
        self.keyboard = Controller()
        self.sub_state_sub = self.create_subscription(SubState, 'submarine_state', self.sub_state_callback, 10)
    
    def sub_state_callback(self, msg):
        if msg.host_mode == 0:
            self.keyboard.press(Key.ctrl)
            self.keyboard.press('c')
            self.keyboard.release('c')
            self.keyboard.release(Key.ctrl)
            sleep(0.5)
            self.keyboard.type('ros2 launch minimal_launch.py')
            self.keyboard.press(Key.enter)
            self.keyboard.release(Key.enter)
    
            
def main(args=None):
    rclpy.init(args=args)
    sub_state_node = Pynput()
    rclpy.spin(sub_state_node)
    rclpy.shutdown()

