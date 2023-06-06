/*  
 *  @author Conner Sommerfield
 *  For questions - Zix on Discord
 *  Controller node for transmitting controller commands on PS4 controller as PID alternative
 *  Uses code very similar to PID such as the thrust_mapper matrix
 *  
 *  To run make sure you source properly and:
 *  1. enable the bluetooth device ds4drv
 *  2. ros2 run joy joy_node
 *  3. ros2 run controller_node controller_exec
 */

#include <vector>
#include <iostream>
#include <stdlib.h>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "vector_operations.hpp"
#include "control_interface.hpp"

#define MOTOR_ID 0x010
#define MOTOR_COUNT 8 // Have to change this to be dynamic
#define MAX_POWER 100

using namespace std;
using std::placeholders::_1;


/* Require:
 * Subscription to ROS built in Joy node
 * CAN Mailbox for CAN Requests
 * Thrust_mapper matrix to take the 6 axis yaw, pitch, roll, x, y, z, and map them to physical movements of
 * the robot that can be implemented using 8 motor thrust values from -100 to 100 
 */
class Controller : public rclcpp::Node
{
  public:
    Controller() 
    : Node("controller")
    {
        controller_sub_ = this->create_subscription<sensor_msgs::msg::Joy>
        ("/joy", 10, std::bind(&Controller::controller_subscription_callback, this, _1));

        can_client_ = this->create_client<scion_types::srv::SendFrame>("send_can_raw");
        

        this->declare_parameter("thrust_mapper", "percy");

        string thrust_mapper = this->get_parameter("thrust_mapper").as_string();
        if (thrust_mapper == "percy")
        {
            this->thrust_mapper_ = Interface::percy_thrust_mapper;
        } 
        else if (thrust_mapper == "junebug")
        {
            this->thrust_mapper_ = Interface::junebug_thrust_mapper;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "GIVE VALID VALUE FOR THRUST MAPPER IN LAUNCH FILE");
            exit(EXIT_FAILURE);
        }

        canClient::setBotInSafeMode(can_client_);
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr  controller_sub_;
    Interface::ros_sendframe_client_t                       can_client_;
    vector<vector<float>>                                   thrust_mapper_;
    

    void controller_subscription_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        vector<float> axes = msg->axes;
        vector<int> buttons = msg->buttons;

        /* These map to the actual controller sticks and buttons */

        /* This part randomly changed on me before pool test, I have no idea why, it was working and then I had to change
        the buttons. Before I had it as 0 5 2 3 4 1, but for the version on the day before pool test it is working
        as 0 4 3 2 5 1 */
        float left_x = -1 * msg->axes[0]; // yaw
        float right_trigger = msg->axes[5] - 1; // pitch
        float left_trigger = msg->axes[2] - 1; // roll
        float right_x = -1 * msg->axes[3]; // x
        float right_y = msg->axes[4]; // y
        float left_y = -1 * msg->axes[1]; // z

        /* Multiply our 8 x 6 mapper matrix by our 6 x 1 ctrl_vals to get an 8 x 1 vector of thrust values (a vector with 8 values) */
        vector<float> ctrl_vals = vector<float>{left_x, right_trigger, left_trigger, right_x, right_y, left_y};
        vector<float> thrust_vals = this->thrust_mapper_ * ctrl_vals;

        make_CAN_request(thrust_vals);
    }

    vector<int> make_CAN_request(vector<float>& thrusts)
    {
        /* Thrusts come out of PID as a float between -1 and 1; motors need int value from -100 to 100 */
        vector<int> convertedThrusts;
        for (float thrust : thrusts)
        {
            convertedThrusts.push_back(((int)(thrust * 100 * MAX_POWER/100)));
        }
        printVector(convertedThrusts);

        /* 
         * We have integer values that are 32 bits (4 bytes) but need values of one byte to send to motor
         * We can extract using an and mask and get last 8 bits which in hex is 0xFF. Char size is one byte
         * which is why we use an array of chars
         */          

        vector<unsigned char> byteThrusts;
        for (int thrust : convertedThrusts)
        {
            byteThrusts.push_back(thrust & 0xFF);
        }
        /* See exactly our 8 thrust values sent to motors */

    ////////////////////////////////////////// BUILD REQUEST //////////////////////////////////////////
        /* 
         * Our frame will send CAN request 
         * one byte for each value -100 to 100 
         */

        canClient::sendFrame(MOTOR_ID, MOTOR_COUNT, byteThrusts.data(), this->can_client_);
        return convertedThrusts;
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}