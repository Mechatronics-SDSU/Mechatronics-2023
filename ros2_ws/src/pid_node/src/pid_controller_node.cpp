/* 
 * @author Conner Sommerfield - Zix on Discord
 * The infamous PID Controller Node
 * Subscribes to all sensor data needed for control system and throws it into a PID object
 * to output ctrl and thrust values.
 * Refer to classes/pid_controller for more info
 * 
 * Every time the subscription callback is triggered, the current state will be updated
 * Current state is stored in member variables of the Node
 * There is a timer that updates every x amount of ms based on what user inputs
 * This calls the PID update function which will use that sensor data to get ctrl/thrust values
 * Then it makes a CAN Request using that array of thrust_vals to send to motors
 * CAN Request is of ID #010
 */

#include <memory>
#include <string>
#include <ctime>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <vector>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "scion_types/msg/desired_state.hpp"
#include "scion_types/msg/orientation.hpp"         // Custom message types defined in scion_types package
#include "scion_types/msg/position.hpp"            
#include "std_msgs/msg/float32.hpp"
#include "scion_pid_controller.hpp"                // PID Class
#include "mbox_can.hpp"                            // For CAN Requests

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;

#define MBOX_INTERFACE "can0"
#define UPDATE_PERIOD 120ms
#define PRINT_PERIOD 500ms

/////////////////////////////////////////////////////////////////////////////////////////////////////
                                    // MEMBER VARIABLE DECLARATIONS // 
/////////////////////////////////////////////////////////////////////////////////////////////////////

/** 
 * Controller Node consists of: 
 *      - timer to update ctrl_vals
 *      - subscription to desired state
 *      - subscription to ms5837 depth data
 *      - subscription to ZED position data
 *      - subscription to AHRS orientation data
 *      - Creates CAN Mailbox and ScionPIDController objects 
**/
class Controller : public rclcpp::Node
{

public:
    Controller(): Node("pid_controller")
    {
        timer_ = this->create_wall_timer(UPDATE_PERIOD, std::bind(&Controller::timer_callback, this));
        
        print_timer_ = this->create_wall_timer(PRINT_PERIOD, std::bind(&Controller::print_timer_callback, this));

        position_sub_ = this->create_subscription<scion_types::msg::Position>
        ("zed_position_data", 10, std::bind(&Controller::position_sub_callback, this, _1));
    
        // depth_sub_ = this->create_subscription<std_msgs::msg::Float32>
        // ("ms5837_depth_data", 10, std::bind(&Controller::depth_sub_callback, this, _1));

        desired_state_sub_ = this->create_subscription<scion_types::msg::DesiredState>
        ("desired_state_data", 10, std::bind(&Controller::desired_state_callback, this, _1));

        orientation_sub_ = this->create_subscription<scion_types::msg::Orientation>
        ("ahrs_orientation_data", 10, std::bind(&Controller::orientation_sub_callback, this, _1));

        // orientation_sub_ = this->create_subscription<scion_types::msg::Orientation>
        // ("zed_orientation_data", 10, std::bind(&Controller::orientation_sub_callback, this, _1));

        // velocity_sub_ = this->create_subscription<scion_types::msg::Orientation>
        // ("dvl_velocity_data", 10, std::bind(&Controller::orientation_sub_callback, this, _1));


        controller_ = Scion_Position_PID_Controller(pid_params_object_.get_pid_params());
        controller_.getStatus();
        
        strncpy(ifr.ifr_name, MBOX_INTERFACE, sizeof(&MBOX_INTERFACE));
        poll_mb_ = new Mailbox::MboxCan(&ifr, "poll_mb");
    }

private:
    rclcpp::Subscription<scion_types::msg::Position>::SharedPtr position_sub_;
    rclcpp::Subscription<scion_types::msg::DesiredState>::SharedPtr desired_state_sub_;
    rclcpp::Subscription<scion_types::msg::Orientation>::SharedPtr orientation_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr depth_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr velocity_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr print_timer_;
    Mailbox::MboxCan* poll_mb_;
    struct ifreq ifr;

    Scion_Position_PID_Controller controller_;
    PID_Params pid_params_object_;                      // Passed to controller for tuning

    /* Upon initialization set all values to [0,0,0...] */

    vector<float> current_orientation_{0.0F, 0.0F, 0.0F};
    vector<float> current_position_{0.0F, 0.0F, 0.0F};
    vector<float> current_state_{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F}; // State described by yaw, pitch, roll, x, y, z 

    vector<float> current_desired_state_; // Desired state is that everything is set to 0 except that its 1 meter below the water {0,0,0,0,0,1}

    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                        // UPDATES OF STATE FOR PIDs // 
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    /* Different timer for printing and sensor updates */
    void print_timer_callback()
    {
        std::cout << "CURRENT AND DESIRED STATE" << "\n" << "___________________________ \n";
        printVector(this->current_state_);
        printVector(this->current_desired_state_);
        std::cout << "___________________________\n\n";
        this->controller_.getStatus(); 
    }

    void timer_callback()
    /* Essential callback set to update PID state at certain interval */
    {
        update_current_state();
    }

    void update_current_state() // ** Important Function **
    {
    /*
     * STEP 1: Build current state from current information 
     * that has been updated from the sensors. As of now
     * we are using orientation as first 3 values, position as next 3
     */

        this->current_state_[0] = this->current_orientation_[0];
        this->current_state_[1] = this->current_orientation_[1];
        this->current_state_[2] = this->current_orientation_[2];
        this->current_state_[3] = this->current_position_[0];
        this->current_state_[4] = this->current_position_[1];
        this->current_state_[5] = this->current_position_[2];
    /* 
     * STEP 2: Update the PID Controller (meaning call the ScionPIDController object's
     * update function which generates ctrl_vals and show its status on the screen 
     */
        if (current_desired_state_.size() == 0)
        {
            current_desired_state_ = current_state_;
        }
        if (current_desired_state_.size() > 0)
        {
                                        // Refer to classes/pid_controller/scion_pid_controller.hpp for this function
            this->controller_.update(current_state_, current_desired_state_, .010); // MOST IMPORTANT LINE IN PROGRAM
        }

    /* STEP 3: Send those generated values to the motors */
        make_CAN_request(this->controller_.current_thrust_values);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                    // CAN REQUESTS FOR MOTOR CONTROL // 
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    void make_CAN_request(vector<float>& thrusts)
    {
        /* Thrusts come out of PID as a float between -1 and 1; motors need int value from -100 to 100 */
        int thrust0 = (int)(thrusts[0]*100);
        int thrust1 = (int)(thrusts[1]*100);
    
        /* See exactly our 8 thrust values sent to motors */
        std::cout << " " << thrust0 << " " << thrust1 << " ";
        std::cout << std::endl;

        /* 
         * We have integer values that are 32 bits (4 bytes) but need values of one byte to send to motor
         * We can extract using an and mask and get last 8 bits which in hex is 0xFF. Char size is one byte
         * which is why we use an array of chars
         */
        unsigned char can_dreq_frame[2] = 
                                {
                                    (thrust0 & 0xFF),
                                    (thrust1 & 0xFF),
                                };             

        // This is a manual motor test can frame that sets each motor to .1 potential
        // char can_dreq_frame[8] = {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10};

    ////////////////////////////////////////// BUILD REQUEST //////////////////////////////////////////
        /* 
         * Our frame will send CAN request 
         * one byte for each value -100 to 100 
         */

        struct can_frame poll_frame;
        poll_frame.can_id = 0x010;
        poll_frame.can_dlc = 2;
        std::copy(std::begin(can_dreq_frame),
                  std::end(can_dreq_frame),
                  std::begin(poll_frame.data));
        if(Mailbox::MboxCan::write_mbox(this->poll_mb_, &poll_frame) == -1) 
        {
            RCLCPP_INFO(this->get_logger(),
            "[DresDecodeNode::_data_request] Failed to write CAN Request.");
	    }
    }
    ////////////////////////////////////////// END REQUEST //////////////////////////////////////////


    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                        // SUBSCRIPTION CALLBACKS // 
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    /* 
     * These callbacks all update a member variable that holds current state of PID, each corresponds to a 
     * sensor. When that sensor publishes, the PID will store the last sensed value  
     */

    void orientation_sub_callback(const scion_types::msg::Orientation::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received ahrs_orientation_data");
        this->current_orientation_ = msg->orientation;
    }

    // void depth_sub_callback(const std_msgs::msg::Float32::SharedPtr msg)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Received ms5837 depth data: %f", msg->data);
    //     this->current_position_ = vector<float>{0.0, 0.0, msg->data} ;
    // }

    void position_sub_callback(const scion_types::msg::Position::SharedPtr msg)
    {    
         RCLCPP_INFO(this->get_logger(), "Received Zed Position Data");
         this->current_position_  = msg->position;
    }
    
    void desired_state_callback(const scion_types::msg::DesiredState::SharedPtr msg)
    /** 
     * You can use this subscription to manually send a desired state at command line
     **/ 
    {
        RCLCPP_INFO(this->get_logger(), "Received Desired Position Data:");
        printVector(msg->desired_state);
        this->current_desired_state_= msg->desired_state; 
    }
};

int main(int argc, char * argv[])
{   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>()); // simply call Controller constructor
    rclcpp::shutdown();
    return 0;
}