'''
    @author Conner Sommerfield
    For questions @Zix on Mechatronics Discord
    DVL Node calls DVL web server implementation using a socket
        Element 0 - yaw
        Element 1 - pitch
        Element 2 - roll
'''

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

import sys
import rclpy
from rclpy.node import Node
from scion_types.msg import State                                            #ToDo: rename to orientation
import socket
import json

class DVL_Node(Node):

    def __init__(self):
        super().__init__('dvl_node')
        self.publisher_ = self.create_publisher(State, 'a50_state_data', 10) #transmits on 'ahrs_orientation' topic
        self.orientation_publisher = self.create_publisher(State, 'ahrs_orientation_data', 10)
        timer_period = .05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.previous_time = 0
        self.time = 0
        
    def get_data(self):
        '''
        Yaw, pitch and roll are all returned as NoneType, must convert to str and then to float
        We can use a formatted float to determine how many decimal places of accuracy we want
        If you want to change precision change all the 2's to whatever precision you want
        '''
        TCP_IP = '192.168.1.4'
        TCP_PORT = 16171
        BUFFER_SIZE = 2048

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((TCP_IP, TCP_PORT))
        data = sock.recv(BUFFER_SIZE)

        json_stream = data
        json_dict = json.loads(json_stream)

        try:
            x = json_dict['x']
            y = json_dict['y']
            z = json_dict['z']
            yaw = json_dict['yaw']
            pitch = json_dict['pitch']
            roll = json_dict['roll']
            try: 
                time = 1 / (json_dict['ts'] - self.previous_time)
            except:
                print('divide by zero error')
            self.previous_time = json_dict['ts']

        except:
            x = 0
            y = 0
            z = 0
            yaw = 0
            pitch = 0
            roll = 0
            # self.time = 0
            
        return x, y, z, yaw, pitch, roll
    
    def timer_callback(self):
        x, y, z, yaw, pitch, roll = self.get_data()

        '''
        Now that we have our data types converted, we can throw the floats into an array which is the format needed for our custom Orientation
        (ToDo: Change this to orientation) message. Then we publish our message using the formatted string below
        '''
        try:
            msg = State()
            msg.state = [x, y, z]
            orientation = State()
            orientation.state = [yaw, pitch, roll]
            self.publisher_.publish(msg)
            self.orientation_publisher.publish(orientation)
            self.get_logger().info('Publishing State Data: "\nx: %f\ny: %f\nz: %f\n"' % (msg.state[0], msg.state[1], msg.state[2]))
            self.get_logger().info('Publishing State Data: "\nyaw: %f\npitch: %f\nroll: %f\n"' % (orientation.state[3], orientation.state[4], orientation.state[5]))

        except:
            pass
        # rclpy.info(time)


def main(args=None):

    rclpy.init(args=args)
    dvl_node = DVL_Node()
    rclpy.spin(dvl_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()