'''
    @author Conner Sommerfield
    For questions @Zix on Mechatronics Discord
    DVL Node calls DVL web server implementation using a socket
'''

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
        timer_period = .05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.previous_time = 0
        
    def get_data(self):
        
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
            try: 
                time = 1 / (json_dict['ts'] - previous_time)
            except:
                print('divide by zero error')
            self.previous_time = json_dict['ts']

        except:
            x = 0
            y = 0
            z = 0
            time = 0
            
        return x, y, z, time
    
    def timer_callback(self):
        x, y, z, time = self.get_data()

        '''
        Now that we have our data types converted, we can throw the floats into an array which is the format needed for our custom Orientation
        (ToDo: Change this to orientation) message. Then we publish our message using the formatted string below
        '''
        msg = State()
        msg.state = [x, y, z]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing State Data: "\nx: %f\ny: %f\nz: %f\n"' % (msg.state[0], msg.state[1], msg.state[2]))
        rclpy.info(time)


def main(args=None):

    rclpy.init(args=args)
    dvl_node = DVL_Node()
    rclpy.spin(dvl_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()