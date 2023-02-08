# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from classes.ahrs_driver_lg import SpartonAHRSDataPackets
from scion_types.srv import Position


class AHRS_Node(Node):

    def __init__(self):
        super().__init__('ahrs_node')
        self.publisher_ = self.create_publisher(Position, 'ahrs', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.ahrs = SpartonAHRSDataPackets()

    def timer_callback(self):
        yaw = self.ahrs.get_true_heading()
        pitch, roll = self.ahrs.get_pitch_roll()
        msg = [0.0, 0.0, 0.0]

        msg.position = [roll, pitch, yaw]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing Orientation Data')
        print(msg.position)


def main(args=None):

    rclpy.init(args=args)
    minimal_publisher = AHRS_Node()
    rclpy.spin(minimal_publisher)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    #ahr = SpartonAHRSDataPackets()
