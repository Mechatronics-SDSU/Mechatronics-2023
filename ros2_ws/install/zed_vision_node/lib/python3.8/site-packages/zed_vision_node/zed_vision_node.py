import sys
sys.path.append("/home/mechatronics/master/Mechatronics-2023/classes/zed_vision")

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Thread
from zed_vision import Zed_Vision
from scion_types.msg import Position


class ZedVision(Node):

    def __init__(self):
        super().__init__('zed_vision_data')
        self.publisher_ = self.create_publisher(Position, 'topic', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.vision = Zed_Vision()
        self.zed, self.opt = self.vision.initCamera()
        capture_thread = Thread(target=self.vision.torch_thread,
                                kwargs={'weights': self.opt.weights, 'img_size': self.opt.img_size, "conf_thres": self.opt.conf_thres})
        capture_thread.start()
        

    def timer_callback(self):
        # something = self.vision.updateCamera(self.zed)
        # print(something)
        
        msg = Position()
        position = self.vision.updateCamera(self.zed)
        if position is not None:
            msg.position = [position[0], position[1], position[2]]      
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing Position Data: "x: %f\ny: %f\nz: %f\n"' % (msg.position[0], msg.position[1], msg.position[2]))


    
def main(args=None):
    rclpy.init(args=args)
    zed_vision_node = ZedVision()

    rclpy.spin(zed_vision_node)
    zed_vision_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()