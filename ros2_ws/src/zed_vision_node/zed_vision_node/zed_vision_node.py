""" 
    @author Gweezy & Zix for questions
    ROS2 Node that publishes vision data
    Vision data implementation with Zed Camera should be defined in classes/zed_vision
    We'll call our endpoint function (update_camera) to get whatever info is returned in the zed_vision class
    I plan on having this be a list of object + the depth map; from there we can implement the logic
    of manipulating this info in other nodes
"""

import sys
# Python syntax is garbage and I appended these for the relative imports
sys.path.append(".")
sys.path.append("..")
sys.path.append("...")
sys.path.append("....")
sys.path.append(".....")
sys.path.append("......")

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Thread
from classes.zed_vision.zed_vision import Zed_Vision
from scion_types.msg import ZedObject
import math

class ZedVision(Node):

    def __init__(self):
        """
        Since the zed_vision class uses a pytorch thread along with the main thread, we have to start this first in our constructor
        We also need to initialize the camera with the initCamera function which will pass in all the parameters we need
        """
        super().__init__('zed_vision_data')
        self.publisher_ = self.create_publisher(ZedObject, 'topic', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.vision = Zed_Vision()
        self.zed, self.opt = self.vision.initCamera()
        capture_thread = Thread(target=self.vision.torch_thread,
                                kwargs={'weights': self.opt.weights, 'img_size': self.opt.img_size, "conf_thres": self.opt.conf_thres})
        capture_thread.start()
        

    def timer_callback(self):
        """
        Here we'll query the zed_vision class for the info using updateCamera function and then publish what we need in ROS messages 
        """
        
        object_list, depth_map = self.vision.updateCamera(self.zed)
        # if object_list:
        #     for object in object_list:
        #         msg = ZedObject()
        #         #msg.label = object.label
        #         msg.velocity = [object.velocity[0]. object.velocity[1], object.velocity[2]]
        #         msg.position = [object.position[0], object.p
        #         position[1], object.position[2]]
        #         self.publisher_.publish(msg)
        #         self.get_logger().info('Publishing Position Data: "x: %f\ny: %f\nz: %f\n"' % (msg.position[0], msg.position[1], msg.position[2]))
        #         self.get_logger().info('Publishing Velocity Data: "x: %f\ny: %f\nz: %f\n"' % (msg.velocity[0], msg.velocity[1], msg.velocity[2]))

        inf = 0
        x = 0
        while x < 720:
            y = 0
            while y < 720:
                depth_value = depth_map.get_value(x,y)
                if math.isinf(depth_value[1]):
                    inf += 1
                # print(str(x) + "," + str(y) + " " + str(depth_value) + "\n")
                # f.write(str(x) + "," + str(y) + " " + str(depth_value) + "\n")
                y += 30
            x += 30
        # f.close()

        if inf > 25:
            return 0
        else:
            return 1

        """
         if position is not None:
            msg.position = [position[0], position[1], position[2]]      
            #msg.
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing Position Data: "x: %f\ny: %f\nz: %f\n"' % (msg.position[0], msg.position[1], msg.position[2]))

        """
       



def main(args=None):
    rclpy.init(args=args)
    zed_vision_node = ZedVision()

    rclpy.spin(zed_vision_node)
    zed_vision_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()