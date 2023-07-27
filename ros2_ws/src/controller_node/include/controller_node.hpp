#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <vector>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "vector_operations.hpp"
#include "control_interface.hpp"

class Controller : public rclcpp::Node
{
    typedef void (Controller::*button_function)();

    public:
        Controller();

    private:
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr      controller_sub_;
        Interface::ros_sendframe_client_t                           can_client_;
        std::vector<std::vector<float>>                             thrust_mapper_;
        std::vector<bool>                                           buttons_;
        std::vector<button_function>                                button_functions_;
        int                                                         motor_count_ = 8;
    

    void controller_subscription_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    void processButtonInputs(std::vector<bool>& button_inputs);
    void killRobot();
    void allClear();
    void turnOnLight();
    void turnOffLight();

    std::vector<float> normalizeCtrlVals(std::vector<float>& ctrl_vals);
    std::vector<int> make_CAN_request(std::vector<float>& thrusts);
};

#endif