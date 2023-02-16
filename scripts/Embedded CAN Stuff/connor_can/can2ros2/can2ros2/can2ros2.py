"""
I am so sorry if you're trying to use this, its only for testing
Don't run this as it's own python program, use "colcon build" and "ros2 run" instead!
"""
import rclpy
from rclpy.node import Node

import can

from std_msgs.msg import Int16MultiArray

class CanDataPublisher(Node):
    """
    Class to listen for an publish CAN frames to a ROS2 topic
    """
    def __init__(self):
        super().__init__('can_publisher')
        self.publisher_ = self.create_publisher(Int16MultiArray, 'can_data', 10) # CAN data is expressed as an array of integers (for now)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period,self.timer_callback)
        self.can_bus = can.interface.Bus(bustype="socketcan",channel="can0", bitrate=500000)
        if self.can_bus:
            self.get_logger().info("Connected to CAN interface.")

    def timer_callback(self):
        try:
            inMsg = self.can_bus.recv()
            if inMsg: #if we get a CAN frame, publish it to Ros2
                pubMsg = Int16MultiArray()
                pubMsg.data = list(inMsg.data)
                pubMsg.data.insert(0,inMsg.arbitration_id)
                self.publisher_.publish(pubMsg)
                self.get_logger().info("Published Recieved CAN Frame :: %s" % str([hex(x) for x in pubMsg.data]))
        except can.CanError:
            self.get_logger().error("Failed to connect to CAN Bus.")

    def destroy_node(self):
        super().destroy_node()
        self.can_bus.shutdown()



def main(args=None):
    rclpy.init(args=args)

    canPublisher = CanDataPublisher()

    rclpy.spin(canPublisher)

    canPublisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()