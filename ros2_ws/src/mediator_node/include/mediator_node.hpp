/* 
 * @author Zix
 * Command Queue Node, translates ideas and puts them into a queue to give a desired state to PIDs
 */

#ifndef MEDIATOR_H
#define MEDIATOR_H

#include <functional>
#include <future>
#include <cmath>
#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <iostream>
#include <unistd.h>
#include <deque>
#include <chrono>

#include "vector_operations.hpp"
#include "scion_types/action/pid.hpp"
#include "scion_types/msg/state.hpp"
#include "scion_types/msg/idea.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "control_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class Mediator : public rclcpp::Node
{
public:
    explicit Mediator();

private:
    Interface::pid_action_client_t          pid_command_client_;
    Interface::ros_trigger_client_t         reset_relative_state_client_;
    Interface::ros_trigger_client_t         reset_relative_position_client_;
    Interface::ros_trigger_client_t         stop_robot_client_;
    Interface::ros_bool_client_t            use_position_client_;
    Interface::ros_bool_client_t            stabilize_robot_client_;
    Interface::idea_sub_t                   idea_sub_;
    Interface::ros_timer_t                  next_command_timer_;
    Interface::ros_timer_t                  reset_timer_;
    Interface::state_sub_t                  current_state_sub_;
    Interface::command_queue_t              command_queue_;
    Interface::int_pub_t                    commands_in_queue_count_pub_;
    Interface::ros_timer_t                  commands_count_timer_;
    Interface::Command*                     current_command_; 
    Interface::current_state_t              current_state_{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};

    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                            // COMMAND HANDLING // 
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    void nextCommandPrep(Interface::state_transform_func command);
    Interface::desired_state_t adjustDesiredState(Interface::desired_state_t& desired_state, Interface::state_transform_func command_function);
    void commandCleanup();

    void nextCommand();

    Interface::current_state_t getCurrentState();
    void addToQueue(Interface::command_vector_t& command_vector);
    void translateIdea(scion_types::msg::Idea::SharedPtr idea);
        /////////////////////////////////////////////////////////////////////////////////////////////////////
                                                // ACTION SERVER // 
        /////////////////////////////////////////////////////////////////////////////////////////////////////


    void send_goal(Interface::desired_state_t& desired);
    void cancel_goal();
    void clearQueue(std::deque<Interface::Command> &q);
    void goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<scion_types::action::PID>::SharedPtr> future);
    void feedback_callback(rclcpp_action::ClientGoalHandle<scion_types::action::PID>::SharedPtr, const std::shared_ptr<const scion_types::action::PID::Feedback> feedback);
    void result_callback(const rclcpp_action::ClientGoalHandle<scion_types::action::PID>::WrappedResult & result);

    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                            // MEDIATION SERVICES // 
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    void stopRobot();
    void resetState();
    void resetPosition();
    void usePositionRequest(bool request);
    void stabilizeRobotRequest(bool request);

    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                            // CALLBACK FUNCTIONS // 
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    void commands_count_timer_callback();
    void current_state_callback(const scion_types::msg::State::SharedPtr msg);

};  // class Mediator

#endif