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
    typedef void (Controller::*button_function)();

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
            motor_count_ = 2;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "GIVE VALID VALUE FOR THRUST MAPPER IN LAUNCH FILE");
            exit(EXIT_FAILURE);
        }

        buttons_ = vector<bool>{false, false, false};
        button_functions_ = vector<button_function>{&Controller::killRobot, &Controller::allClear, &Controller::turnOnLight};

        canClient::setBotInSafeMode(can_client_);
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr  controller_sub_;
    Interface::ros_sendframe_client_t                       can_client_;
    vector<vector<float>>                                   thrust_mapper_;
    int                                                     motor_count_ = 8;
    vector<bool>                                            buttons_;
    vector<button_function>                                 button_functions_;
    

    void controller_subscription_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        vector<float> axes = msg->axes;
        vector<int> buttons = msg->buttons;

        /* These map to the actual controller sticks and buttons */

        /* This part randomly changed on me before pool test, I have no idea why, it was working and then I had to change
        the buttons. Before I had it as 0 5 2 3 4 1, but for the version on the day before pool test it is working
        as 0 4 3 2 5 1 */
        float left_x        = -1 *  msg->axes[0];           // yaw
        float right_trigger =       msg->axes[5] - 1;       // pitch
        float left_trigger  =       msg->axes[2] - 1;       // roll
        float right_x       = 1 *   msg->axes[4];           // x
        float right_y       = -1 *  msg->axes[3];           // y
        float left_y        = -1 *  msg->axes[1];           // z
        
        /* Multiply our 8 x 6 mapper matrix by our 6 x 1 ctrl_vals to get an 8 x 1 vector of thrust values (a vector with 8 values) */

        vector<float> ctrl_vals = vector<float>{left_x, right_trigger, left_trigger, right_x, right_y, left_y};
        ctrl_vals = normalizeCtrlVals(ctrl_vals);
        vector<float> thrust_vals = this->thrust_mapper_ * ctrl_vals;

        bool x_button = msg->buttons[0];
        bool o_button = msg->buttons[1];
        bool tri_button = msg->buttons[2];
        vector<bool> button_vals{x_button, o_button, tri_button};

        make_CAN_request(thrust_vals);
        processButtonInputs(button_vals);
    }


    // 0: x, 1: o, 2: tri, 3: square
    void processButtonInputs(vector<bool>& button_inputs)
    {
        for (int i = 0; i < button_inputs.size(); i++)
        {
            if (button_inputs[i] && !this->buttons_[i]) 
            {
                this->buttons_[i] = 1;
                (this->*(button_functions_[i]))();
            }
            if (!button_inputs[i] && this->buttons_[i]) 
            {
                this->buttons_[i] = 0;
            }
        }
    }

    void killRobot()
    {
        canClient::sendFrame(0x00, 0, 0, this->can_client_);
    }

    void allClear()
    {
        canClient::sendFrame(0x00A, 0, 0, this->can_client_);
    }

    void turnOnLight()
    {
        vector<unsigned char> lightEnable{0x04, 0x00, 0x00, 0x00, 0x01};
        canClient::sendFrame(0x22, 5, lightEnable.data(), this->can_client_);
        vector<unsigned char> lightOn{0x04, 0x00, 0x04, 0x00, 0x64};
        canClient::sendFrame(0x22, 5, lightOn.data(), this->can_client_);
    }

    vector<float> normalizeCtrlVals(vector<float>& ctrl_vals)
    {
        vector<float> normalized{0,0,0,0,0,0};
        
        float vectorTotal = abs(ctrl_vals[0]) + abs(ctrl_vals[3]) + abs(ctrl_vals[4]); // yaw, x, y
        float nonVectorTotal = abs(ctrl_vals[1]) + abs(ctrl_vals[2]) + abs(ctrl_vals[5]); // roll, pitch, z

        normalized[0] = ctrl_vals[0];
        normalized[1] = ctrl_vals[1];
        normalized[2] = ctrl_vals[2];
        normalized[3] = ctrl_vals[3];
        normalized[4] = ctrl_vals[4];
        normalized[5] = ctrl_vals[5];

        if (vectorTotal > 1)
        {
            normalized[0] = ctrl_vals[0] / vectorTotal;
            normalized[3] = ctrl_vals[3] / vectorTotal;
            normalized[4] = ctrl_vals[4] / vectorTotal;
        }
        if (nonVectorTotal > 1)
        {
            normalized[1] = ctrl_vals[1] / nonVectorTotal;
            normalized[2] = ctrl_vals[2] / nonVectorTotal;
            normalized[5] = ctrl_vals[5] / nonVectorTotal;
        }

        if (vectorTotal == 0)
        {
            normalized[0] = 0;
            normalized[3] = 0;
            normalized[4] = 0;
        }

        if (nonVectorTotal == 0)
        {
            normalized[1] = 0;
            normalized[2] = 0;
            normalized[5] = 0;
        }

        return normalized;
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

        canClient::sendFrame(MOTOR_ID, motor_count_, byteThrusts.data(), this->can_client_);
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