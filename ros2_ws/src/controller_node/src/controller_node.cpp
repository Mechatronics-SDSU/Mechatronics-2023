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
                                        {-1, -1,  1,  0,  0,  1},
                                        { 0,  0,  0,  1,  1,  0},
                                        {-1,  1,  1,  0,  0,  1},
                                        { 0,  0,  0,  1, -1,  0}
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

        float left_x = msg->axes[0]; // yaw
        float right_trigger = msg->axes[5] - 1; // pitch
        float left_trigger = msg->axes[2] - 1; // roll
        float right_x = msg->axes[3]; // x
        float right_y = msg->axes[4]; // y
        float left_y = msg->axes[1]; // z

        vector<float> ctrl_vals = vector<float>{left_x, right_trigger, left_trigger, right_x, right_y, left_y};
        vector<float> thrust_vals = this->thrust_mapper_ * ctrl_vals;

        make_CAN_request(thrust_vals);
    }


    void make_CAN_request(vector<float> thrusts)
    {
        /* Thrusts come out of PID as a float between -1 and 1; motors need int value from -100 to 100 */
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