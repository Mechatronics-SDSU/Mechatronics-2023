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

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "mbox_can.hpp" 
#include "vector_operations.hpp"

#define MBOX_INTERFACE "can0"

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

        strncpy(ifr.ifr_name, MBOX_INTERFACE, sizeof(&MBOX_INTERFACE));
        poll_mb_ = new Mailbox::MboxCan(&ifr, "poll_mb");

        thrust_mapper_ = vector<vector<float>>
                                    {
                                        { 0,  1, -1,  0,  0,  1},                   
                                        { 1,  0,  0,  1,  1,  0},
                                        { 0, -1, -1,  0,  0,  1},
                                        { 1,  0,  0,  1, -1,  0},
                                        { 0, -1,  1,  0,  0,  1},
                                        {-1,  0,  0,  1,  1,  0},
                                        { 0,  1,  1,  0,  0,  1},
                                        {-1,  0,  0,  1, -1,  0}
                                    };
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr controller_sub_;
    Mailbox::MboxCan* poll_mb_;
    struct ifreq ifr;
    vector<vector<float>> thrust_mapper_;
    
    
    void controller_subscription_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        vector<float> axes = msg->axes;
        vector<int> buttons = msg->buttons;

        /* These map to the actual controller sticks and buttons */

        /* This part randomly changed on me before pool test, I have no idea why, it was working and then I had to change
        the buttons. Before I had it as 0 5 2 3 4 1, but for the version on the day before pool test it is working
        as 0 4 3 2 5 1 */
        float left_x = msg->axes[0]; // yaw
        float right_trigger = msg->axes[5] - 1; // pitch
        float left_trigger = msg->axes[2] - 1; // roll
        float right_x = msg->axes[4]; // x
        float right_y = msg->axes[3]; // y
        float left_y = -1 * msg->axes[1]; // z

        /* Multiply our 8 x 6 mapper matrix by our 6 x 1 ctrl_vals to get an 8 x 1 vector of thrust values (a vector with 8 values) */
        vector<float> ctrl_vals = vector<float>{left_x, right_trigger, left_trigger, right_x, right_y, left_y};
        vector<float> thrust_vals = this->thrust_mapper_ * ctrl_vals;

        make_CAN_request(thrust_vals);
    }


    void make_CAN_request(vector<float> thrusts)
    {
        /* Thrusts come out of PID as a float between -1 and 1; motors need int value from -100 to 100 and scale */
        int thrust0 = (int)(thrusts[0]*100/3);
        int thrust1 = (int)(thrusts[1]*100/3);
        int thrust2 = (int)(thrusts[2]*100/3);
        int thrust3 = (int)(thrusts[3]*100/3);
        int thrust4 = (int)(thrusts[4]*100/3);
        int thrust5 = (int)(thrusts[5]*100/3);
        int thrust6 = (int)(thrusts[6]*100/3);
        int thrust7 = (int)(thrusts[7]*100/3);

        /* See exactly our 8 thrust values sent to motors */
        std::cout << " " << thrust0 << " " << thrust1 << " " << thrust2 << " " << thrust3;
        std::cout << " " << thrust4 << " " << thrust5 << " " << thrust6 << " " << thrust7;
        std::cout << std::endl;

        /* 
         * We have integer values that are 32 bits (4 bytes) but need values of one byte to send to motor
         * We can extract using an and mask and get last 8 bits which in hex is 0xFF. Char size is one byte
         * which is why we use an array of chars
         */
        unsigned char can_dreq_frame[8] = 
                                {
                                    (thrust0 & 0xFF),
                                    (thrust1 & 0xFF),
                                    (thrust2 & 0xFF),
                                    (thrust3 & 0xFF),
                                    (thrust4 & 0xFF),
                                    (thrust5 & 0xFF),
                                    (thrust6 & 0xFF),
                                    (thrust7 & 0xFF) 
                                };             

        // This is a manual motor test can frame that sets each motor to .1 potential
        // char can_dreq_frame[8] = {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10};

    /********************************************* BUILD REQUEST ********************************************/
        /* 
         * Our frame will send CAN request 
         * one byte for each value -100 to 100 
         */

        struct can_frame poll_frame;
        poll_frame.can_id = 0x010;
        poll_frame.can_dlc = 8;
        std::copy(std::begin(can_dreq_frame),
                  std::end(can_dreq_frame),
                  std::begin(poll_frame.data));
        if(Mailbox::MboxCan::write_mbox(this->poll_mb_, &poll_frame) == -1) 
        {
            RCLCPP_INFO(this->get_logger(),
            "[DresDecodeNode::_data_request] Failed to write DREQ.");
	    }

    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}