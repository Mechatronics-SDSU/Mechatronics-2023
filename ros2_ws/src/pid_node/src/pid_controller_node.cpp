#include <memory>
#include <string>
#include <ctime>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "scion_types/msg/orientation.hpp"         // Custom message types defined in scion_types package
#include "scion_types/msg/position.hpp"         // Custom message types defined in scion_types package
#include "scion_pid_controller.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;


/** 
 *  Node sublcass will act as the listener/responder of our node pair that will wait for talker 
 *  Responsible for printing unixTime and converting to humanTime, then sending date back to Node1
**/
class Controller : public rclcpp::Node
{

public:
    /** 
     * Create a subscriber 
     **/
    Controller(): Node("pid_controller")
    {

        /* Must create subscsription to all data needed for PID to include current position, current orientation, current velocity, and desired position/orientation/velocity */
        /* position_sub_ = this->create_subscription<scion_types::msg::Position>
        ("position_data", 10, std::bind(&Controller::position_sub_callback, this, _1));
        desired_position_sub_ = this->create_subscription<scion_types::msg::Position>
        ("desired_position", 10, std::bind(&Controller::desired_position_sub_callback, this, _1)); */
        orientation_sub_ = this->create_subscription<scion_types::msg::Orientation>
        ("ahrs_orientation", 10, std::bind(&Controller::orientation_sub_callback, this, _1));
        controller_ = Scion_Position_PID_Controller(pid_params_object_.get_pid_params());
        controller_.getStatus();
        // 
        
    }

private:

    rclcpp::Subscription<scion_types::msg::Position>::SharedPtr position_sub_;
    rclcpp::Subscription<scion_types::msg::Position>::SharedPtr desired_position_sub_;
    rclcpp::Subscription<scion_types::msg::Orientation>::SharedPtr orientation_sub_;
    Scion_Position_PID_Controller controller_;
    PID_Params pid_params_object_;

    vector<float> current_orientation_{0.0F,0.0F,0.0F};
    vector<float> current_position_{0.0F,0.0F,0.0F};

    vector<float> current_state_{0.0F,0.0F,0.0F,0.0F,0.0F,0.0F};
    vector<float> current_desired_state_{0.0F,0.0F,0.0F,0.0F,0.0F,0.0F};

    void update_current_state()
    {
        this->current_state_[0] = this->current_orientation_[0];
        this->current_state_[1] = this->current_orientation_[1];
        this->current_state_[2] = this->current_orientation_[2];

        this->current_state_[3] = this->current_position_[0];
        this->current_state_[4] = this->current_position_[1];
        this->current_state_[5] = this->current_position_[2];

        RCLCPP_INFO(this->get_logger(), "Current State:");
        this->controller_.getStatus();
        printVector(this->current_state_);
    }


    void orientation_sub_callback(const scion_types::msg::Orientation::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received ahrs_orientation_data:");
        // printVector(msg->orientation);
        this->current_orientation_ = msg->orientation;
        this->controller.update(current_state_, current_desired_state_, .010);
        update_current_state();
    }

    void position_sub_callback(const scion_types::msg::Position::SharedPtr msg)
    /** 
     * .
     **/ 
    {
         RCLCPP_INFO(this->get_logger(), "Received Sensor Data:");
         printVector(msg->position);
         RCLCPP_INFO(this->get_logger(), "Error:");
         printVector(getError(this->current_desired_state_, msg->position));
         this->current_position_ = msg->position;
    }

    void desired_position_sub_callback(const scion_types::msg::Position::SharedPtr msg)
    /** 
     * 
     **/ 
    {
         RCLCPP_INFO(this->get_logger(), "Received Desired Position Data:");
         printVector(msg->position);
         this->current_desired_state_sub_ = msg->position; 
    }

    void printVector(vector<float> floatVector)
    {
        for (float number : floatVector)
        {
            std::cout << number << " " << endl;
        }
    }

    vector<float> getError(vector<float> desired_position, vector<float> current_position) 
    {
        vector<float> errorVector{0.0F,0.0F,0.0F,0.0F,0.0F,0.0F};
        for (auto i = 0; i < (float)desired_position.size(); i++) 
        {
            errorVector[i] = desired_position[i] - current_position[i];
        }
        return errorVector;
    }
};

int main(int argc, char * argv[])
{   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>()); // simply call Controller constructor
    rclcpp::shutdown();
    return 0;
}
