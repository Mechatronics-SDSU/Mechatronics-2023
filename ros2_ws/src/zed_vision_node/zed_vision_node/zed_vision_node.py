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

import rclpy
from rclpy.node import Node
from threading import Thread
from classes.zed_vision.zed_vision import Zed_Vision
from scion_types.msg import Idea
from scion_types.msg import ZedObject
from scion_types.msg import VisionObject
from scion_types.msg import Position
from scion_types.msg import Keypoint2Di
import math
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

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

coeffs =  [
    0.090011231804371705,
    0.090548754148121260,
    0.090968151967928082,
    0.091268433002908766,
    0.091448886490887449,
    0.091509085171565405,
    0.091448886490887449,
    0.091268433002908766,
    0.090968151967928082,
    0.090548754148121260,
    0.090011231804371705
]

class Smoother:
    def __init__(self, points, window_size):
        self.points = []
        for i in range (0, points):
            self.points.append([0] * window_size)
        
        # self.x_avg_fir = [0] * window_size
        # self.y_avg_fir = [0] * window_size
    def smooth(self, static_list, coeffs, in_x):
        tmp_x = 0
        static_list.pop()
        static_list.insert(0, in_x)
        for index, value in enumerate(static_list):
            tmp_x += coeffs[index] * value
        return tmp_x
    


class ZedVision(Node):

    def __init__(self):
        """
        Since the zed_vision class uses a pytorch thread along with the main thread, we have to start this first in our constructor
        We also need to initialize the camera with the initCamera function which will pass in all the parameters we need
        """
        super().__init__('zed_vision_data')
        self.vision_object_publisher = self.create_publisher(VisionObject, 'zed_object_data', 10)
        self.vision_publisher = self.create_publisher(ZedObject, 'zed_vision_data', 10)
        self.position_publisher = self.create_publisher(Position, 'zed_position_data', 10)
        self.idea_publisher = self.create_publisher(Idea, 'brain_idea_data', 10)
        self.smoother = Smoother(10, len(coeffs))
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
        # self.idea_publisher.publish(idea)


    def timer_callback(self):
        """
        Here we'll query the zed_vision class for the info using updateCamera function and then publish what we need in ROS messages 
        """
        
        object_list, depth_map, zed_pose, py_translation, vision_object_list = self.vision.updateCamera(self.zed)

        self.processDepthData(depth_map)    
        tx =  1 * round(zed_pose.get_translation(py_translation).get()[0], 3)
        ty = round(zed_pose.get_translation(py_translation).get()[1], 3)
        tz = -1 * round(zed_pose.get_translation(py_translation).get()[2], 3)

        position = Position()
        position.position = [tz, tx, ty]
        self.position_publisher.publish(position)
        # self.get_logger().info('Publishing Position Data:\n "x: %f\ny: %f\nz: %f\n"' % (tz,tx,ty))

        if object_list:
            for object in object_list:
                msg = ZedObject()
                msg.label_id = object.raw_label
                msg.position = [object.position[0], object.position[1], object.position[2]]
                corners = []
                for point in object.bounding_box_2d:
                    kp = Keypoint2Di()
                    kp.kp = []
                    kp.kp.append(int(point[0]))
                    kp.kp.append(int(point[1]))
                    corners.append(kp)
                msg.corners = corners

                    # x = [object.bounding_box_2d[0][0], object.bounding_box_2d[1][0], object.bounding_box_2d[2][0], object.bounding_box_2d[3][0], (object.bounding_box_2d[0][0] + object.bounding_box_2d[1][0]) / 2]
                    # y = [object.bounding_box_2d[0][1], object.bounding_box_2d[1][1], object.bounding_box_2d[2][1], object.bounding_box_2d[3][1], (object.bounding_box_2d[0][1] + object.bounding_box_2d[2][1]) / 2]
                    # smooth_x = []
                    # smooth_y = []
                    # for index, x_point in enumerate(x):
                    #     tmp_x = self.smoother.smooth(self.smoother.points[index], coeffs, x_point)
                    #     smooth_x.append(tmp_x)
                    # for index, y_point in enumerate(y):
                    #     tmp_y = self.smoother.smooth(self.smoother.points[index + 5], coeffs, y_point)
                    #     smooth_y.append(tmp_y)

                    # x = (object.bounding_box_2d[0][0] + object.bounding_box_2d[3][0]) / 2
                    # y = 720 - ((object.bounding_box_2d[0][1] + object.bounding_box_2d[2][1]) / 2)
                    # smooth_x = self.smoother.smooth(self.smoother.x_avg_fir, coeffs, x)
                    # smooth_y = self.smoother.smooth(self.smoother.y_avg_fir, coeffs, y)
                    # print(x,y)

                    # matplotlib.use('TkAgg')
                    # plt.axis([0, 1280, 0, 720])
                    # plt.plot(smooth_x,smooth_y,'b*', markersize=20)
                    # plt.plot(x,y,'r*', markersize=20)
                    # plt.grid()
                    # plt.ion()
                    # plt.show()                                                                  # [0,0], [1,1], [], []
                    # plt.pause(.01)
                    # plt.clf()

                    # Recomment to here - joseph
                    #print(object.bounding_box_2d)
                    #corners = []

                    # print(object.bounding_box_2d)
                    ##self.vision_publisher.publish(msg)
        
                    
                    # print(object.bounding_box_2d)
                self.vision_publisher.publish(msg)
        

        if vision_object_list:
            for object in vision_object_list:
                msg = VisionObject()
                msg.object_name = object.object_name
                msg.distance = object.distance
                self.vision_object_publisher.publish(msg)


import subprocess

import numpy as np
from scipy.signal import butter,filtfilt
def butter_lowpass_filter(data, cutoff, fs, order):
    nyq = 0.5 * fs  # Nyquist Frequency
    normal_cutoff = cutoff / nyq
    # Get the filter coefficients 
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data)
    return y

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
