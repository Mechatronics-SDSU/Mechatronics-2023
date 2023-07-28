#ifndef CURRENT_STATE_H
#define CURRENT_STATE_H

#include <memory>
#include <vector>
#include <unistd.h>
#include <thread>
#include <future>
#include <iostream>
#include <functional>
#include <math.h>
#include <string>

#include "vector_operations.hpp"
#include "rclcpp/rclcpp.hpp"
#include "control_interface.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "scion_types/msg/state.hpp"
#include "scion_types/msg/position.hpp"
#include "scion_types/msg/orientation.hpp"
#include "scion_types/msg/datapoint.hpp"
#include "std_msgs/msg/float32.hpp"

/* Subscribe to all relevant sensor information and consolidate it for the PID Node to subscribe to */

class CurrentStateNode : public rclcpp::Node
{
  public:
    CurrentStateNode();

  // private:    
    Interface::position_sub_t           position_sub_;
    Interface::state_sub_t              orientation_sub_;
    Interface::datapoint_sub_t          depth_sub_;
    Interface::state_pub_t              absolute_state_pub_;
    Interface::state_pub_t              relative_state_pub_;
    Interface::ros_trigger_service_t    reset_relative_state_service_;
    Interface::ros_trigger_service_t    reset_relative_position_service_;
    Interface::state_sub_t              dvl_position_sub_;
    Interface::state_sub_t              a50_state_sub_;
    Interface::ros_timer_t              state_pub_timer_;
    Interface::current_state_t          current_state_  {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
    Interface::current_state_t          relative_state_ {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
    std::vector<float>                  current_orientation_{0.0F, 0.0F, 0.0F};
    std::vector<float>                  current_position_   {0.0F, 0.0F, 0.0F};
    bool current_state_valid_ =         false;
    bool orientation_valid_ =           true;
    bool position_valid_ =              true;
    
    void initCurrentState();
    bool currentStateValid();

    void update_current_state();

    void publish_absolute_state();
    void publish_relative_state();
    void publish_state();

    void resetRelativeState (const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void resetRelativePosition (const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                        // SUBSCRIPTION CALLBACKS // 
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    /* 
     * These callbacks all update a member variable that holds current state of PID, each corresponds to a 
     * sensor. When that sensor publishes, the PID will store the last sensed value  
     */

    void orientation_sub_callback(const scion_types::msg::State::SharedPtr msg);

    void a50_state_sub_callback(const scion_types::msg::State::SharedPtr msg);
};

#endif
