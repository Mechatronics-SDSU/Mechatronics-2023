'''
    @author Conner Sommerfield
    For questions @Zix on Mechatronics Discord
    AHRS Node calls AHRS class to publish a float32[] array that contains orientation of the scion bot.
    Published every .6 seconds as AHRS DocSheet mentions an accuracy for reporting 100 times a second (60/100 = .6)
    Subscribe to 'ahrs_orientation' topic to grab orientation info. This wil be in the form of an array:
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
sys.path.append('/home/mechatronics/Mechatronics-2023/classes/ahrs_driver_lg')

import rclpy
from rclpy.node import Node
from ahrs import SpartonAHRSDataPackets
from scion_types.msg import Position                                            #ToDo: rename to orientation


class AHRS_Node(Node):


    def __init__(self):
        super().__init__('ahrs_node')
        self.publisher_ = self.create_publisher(Position, 'ahrs_orientation', 10)     #transmits on 'ahrs_orientation' topic
        timer_period = 0.6  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.ahrs = SpartonAHRSDataPackets()
        
    
    def format_orientation(yaw, pitch, roll):
        yawFormatted = yaw
        pitchFormatted = pitch
        rollFormatted = roll

        if yawFormatted != None:
            yawFormatted = round(float(str(yawFormatted)), 2)
        if pitchFormatted != None:
            pitchFormatted = round(float(str(pitchFormatted)), 2)
        if rollFormatted != None:
            rollFormatted = round(float(str(rollFormatted)), 2)
            
        return yawFormatted, pitchFormatted, rollFormatted


    def timer_callback(self):
        yaw = self.ahrs.get_true_heading()
        pitch, roll = self.ahrs.get_pitch_roll()

        '''Yaw, pitch and roll are all returned as NoneType, must convert to str and then to float
        We can use a formatted float to determine how many decimal places of accuracy we want
        If you want to change precision change all the 2's to whatever precision you want'''

        yaw, pitch, roll = format_orientation(yaw, pitch, roll)

        '''Now that we have our data types converted, we can throw the floats into an array which is the format needed for our custom position (ToDo: Change this to orientation) message. Then we publish our message using the formatted string below'''
        if (yaw != None and pitch != None and roll != None):
            msg = Position()
            msg.position = [yaw, pitch, roll]
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing Orientation Data: "\nyaw: %f\npitch: %f\nroll: %f\n"' % (msg.position[0], msg.position[1], msg.position[2]))


def main(args=None):

    rclpy.init(args=args)
    ahrs_node = AHRS_Node()
    rclpy.spin(ahrs_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()


'''Archive # if yaw != None:
        #     print(f"yaw: {float(str(yaw)):.5f}")
        # if pitch != None:
        #     print(f"pitch: {float(str(pitch)):.5f}")
        # if roll != None:
        #     print(f"roll: {float(str(roll)):.5f}")


          # print(f"yaw:", (str(yaw)))
        # print(f"pitch:", (str(pitch)))
        # print(f"roll:", (str(roll)))


        # print(yawFormatted)
        # print(pitchFormatted)
        # print(rollFormatted)

        # 
'''