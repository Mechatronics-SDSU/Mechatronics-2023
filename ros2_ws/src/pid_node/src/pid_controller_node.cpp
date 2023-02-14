#include <memory>
#include <string>
#include <ctime>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <vector>

#include "rclcpp/rclcpp.hpp"
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
        position_sensor_ = this->create_subscription<scion_types::msg::Position>
        ("position_data", 10, std::bind(&Controller::position_sensor_callback, this, _1));
        desired_position_ = this->create_subscription<scion_types::msg::Position>
        ("desired_position", 10, std::bind(&Controller::desired_position_callback, this, _1));
    }

private:

    rclcpp::Subscription<scion_types::msg::Position>::SharedPtr position_sensor_;
    rclcpp::Subscription<scion_types::msg::Position>::SharedPtr desired_position_;
    vector<float> current_position_{0.0,0.0,0.0,0.0,0.0,0.0};
    vector<float> current_desired_position_{0.0,0.0,0.0,0.0,0.0,0.0};

    void position_sensor_callback(const scion_types::msg::Position::SharedPtr msg)
    /** 
     * Subscriber binded function prints the unixTime whenever the publisher from Node1 sends it.
     **/ 
    {
         RCLCPP_INFO(this->get_logger(), "Received Sensor Data:");
         printVector(msg->position);
         RCLCPP_INFO(this->get_logger(), "Error:");
         printVector(getError(this->current_desired_position_, msg->position));
         this->current_position_ = msg->position;
    }

    void desired_position_callback(const scion_types::msg::Position::SharedPtr msg)
    /** 
     * Subscriber binded function prints the unixTime whenever the publisher from Node1 sends it.
     **/ 
    {
         RCLCPP_INFO(this->get_logger(), "Received Desired Position Data:");
         printVector(msg->position);
         this->current_desired_position_ = msg->position; 
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
        vector<float> errorVector{0.0,0.0,0.0,0.0,0.0,0.0};
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
