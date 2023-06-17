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
sys.path.append("/home/mechatronics/master")

import statistics
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Thread
from classes.zed_vision.zed_vision import Zed_Vision
from scion_types.msg import Idea
from scion_types.msg import ZedObject
from scion_types.msg import Position
import math


# Defines a box of pixels of interest for current camera frame
# Variables here should be immutable
XSTART = 360
XEND = 600
YSTART = 450
YEND = 890
PIXEL_WINDOW = (XEND - XSTART) * (YEND - YSTART)
THRESHOLD_DISTANCE = .55
INFTHRESHOLD = 10
PIXELTHRESHOLD = 10

class ZedVision(Node):

    def __init__(self):
        """
        Since the zed_vision class uses a pytorch thread along with the main thread, we have to start this first in our constructor
        We also need to initialize the camera with the initCamera function which will pass in all the parameters we need
        """
        super().__init__('zed_vision_data')
        self.vision_publisher = self.create_publisher(ZedObject, 'topic', 10)
        self.position_publisher = self.create_publisher(Position, 'zed_position_data', 10)
        self.idea_publisher = self.create_publisher(Idea, 'brain_idea_data', 10)
        timer_period = .05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.vision = Zed_Vision()
        self.zed, self.opt = self.vision.initCamera()
        capture_thread = Thread(target=self.vision.torch_thread,
                                kwargs={'weights': self.opt.weights, 'img_size': self.opt.img_size, "conf_thres": self.opt.conf_thres})
        capture_thread.start()
        
                       
    def iterate_pixels(self, depth_ocv) -> int:
        inf = 0
        lessThan = 0
        x = XSTART
        while x < XEND:
            y = YSTART
            while y < YEND:
                if math.isinf(depth_ocv[x,y]):
                    inf += 1
                if depth_ocv[x,y] < THRESHOLD_DISTANCE:
                    lessThan += 1
                y += 5
            x += 5
        return inf, lessThan


    def processDepthData(self, depth_map):
        depth_ocv = depth_map.get_data()
        inf, lessThan = self.iterate_pixels(depth_ocv)

        if inf > INFTHRESHOLD or lessThan > PIXELTHRESHOLD:
            self.stop_idea()


    def stop_idea(self):
        idea = Idea()
        idea.code = 0
        self.idea_publisher.publish(idea)


    def timer_callback(self):
        """
        Here we'll query the zed_vision class for the info using updateCamera function and then publish what we need in ROS messages 
        """
        
        object_list, depth_map, zed_pose, py_translation = self.vision.updateCamera(self.zed)
        # if object_list:
        #     for object in object_list:
        #         msg = ZedObject()
        #         print(type(object.bounding_box))
        #         print(object.bounding_box)

                # msg.label = object.label
                # msg.velocity = [object.velocity[0]. object.velocity[1], object.velocity[2]]
                # msg.position = [object.position[0], object.position[1], object.position[2]]
                # self.publisher_.publish(msg)
                # print(msg.label)
                # print(object.id)
                # print(object.label_id)
                # print(object.position)
                # self.get_logger().info('Publishing Position Data: "[x: %fy: %fz: %f]"' % (msg.position[0], msg.position[1], msg.position[2]))
                # self.get_logger().info('Publishing Velocity Data: "x: %f\ny: %f\nz: %f\n"' % (msg.velocity[0], msg.velocity[1], msg.velocity[2]))

        """ Wall/Object avoidance code in progress """        

        self.processDepthData(depth_map)    
        tx =  1 * round(zed_pose.get_translation(py_translation).get()[0], 3)
        ty = round(zed_pose.get_translation(py_translation).get()[1], 3)
        tz = -1 * round(zed_pose.get_translation(py_translation).get()[2], 3)

        position = Position()
        position.position = [tz, tx, ty]
        self.position_publisher.publish(position)
        self.get_logger().info('Publishing Position Data:\n "x: %f\ny: %f\nz: %f\n"' % (tz,tx,ty))

    

import subprocess

def GUI(depth_map):
        import matplotlib
        matplotlib.use('TkAgg')
        import matplotlib.pyplot as plt
        import numpy as np

        from numpy import inf

        depth_ocv = depth_map.get_data()

        depth_ocv = np.clip(depth_ocv, 0, 1)

        # depth_ocv = depth_data[512:768, 360:720]
        np.round(depth_ocv, decimals=1)

        # depth_ocv[depth_ocv] = 10

        # depth_ocv[np.isposinf(depth_ocv)] = 10
        # depth_ocv[np.isneginf(depth_ocv)] = -10

        plt.imshow(depth_ocv)
        plt.show()
        # print(depth_ocv)
        

def move():
    bashCommand = "cansend can0 010#1010"
    subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)       

def turn():
    bashCommand = "cansend can0 010#05F1"
    subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)       

def doNothing():
    bashCommand = "cansend can0 010#0000"
    subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE) 
    bashCommand = "cansend can0 00A#"
    subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE) 

def main(args=None):
    rclpy.init(args=args)
    zed_vision_node = ZedVision()

    rclpy.spin(zed_vision_node)
    zed_vision_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
