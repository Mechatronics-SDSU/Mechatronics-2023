#ifndef PID_H
#define PID_H

#include <memory>
#include <string>
#include <ctime>
#include <thread>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <vector>
#include <cstring>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_interface.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "scion_types/action/pid.hpp"
#include "scion_types/msg/state.hpp"
#include "scion_pid_controller.hpp"                // PID Class
#include "pid_controller.hpp"   

/////////////////////////////////////////////////////////////////////////////////////////////////////
                                    // MEMBER VARIABLE DECLARATIONS // 
/////////////////////////////////////////////////////////////////////////////////////////////////////

class Controller : public rclcpp::Node
{
public:
    explicit Controller();

// private:
    Interface::ros_timer_t                      update_timer_;
    Interface::ros_timer_t                      print_timer_;
    Interface::state_sub_t                      current_state_sub_;
    Interface::state_sub_t                      desired_state_sub_;
    Interface::tune_pid_sub_t                   kp_tuning_sub_;
    Interface::tune_pid_sub_t                   ki_tuning_sub_;
    Interface::tune_pid_sub_t                   kd_tuning_sub_;
    Interface::pid_action_server_t              pid_command_server_;
    Interface::ros_bool_service_t               use_position_service_;
    Interface::ros_bool_service_t               stabilize_robot_service_;
    Interface::ros_trigger_service_t            stop_robot_service_;
    Interface::ros_trigger_client_t             reset_relative_state_client_;        
    Interface::ros_trigger_client_t             reset_relative_position_client_;
    Interface::ros_sendframe_client_t           can_client_;
    Interface::matrix_t                         thrust_mapper_;
    Scion_Position_PID_Controller               controller_;
    PID_Params                                  pid_params_object_;                      // Passed to controller for tuning
    Interface::ros_trigger_client_t             pid_ready_client_;
    std::map<int, string>                       axis_tuning_map_;
    int                                         motor_count_ = 8;

    /* Upon initialization set all values to [0,0,0] */
    
    Interface::current_state_t current_state_{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F}; // State described by yaw, pitch, roll, x, y, z 
    Interface::desired_state_t desired_state_{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F}; // Desired state is that everything is set to 0 except that its 1 meter below the water {0,0,0,0,0,1}
    std::vector<unsigned char> nothing_ = std::vector<unsigned char> {0x00}; // Commands over 100 don't change motor power
    bool current_state_valid_ = false;
    bool desired_state_valid_ = false;
    bool use_position_ = true;
    bool service_done_ = false;
    bool stabilize_robot_ = true;

    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                // WAIT FOR VALID DATA TO INITIALIZE PIDs // 
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    void initDesiredState();
    bool desiredStateValid();
    void printCurrentAndDesiredStates();
    void sendNothingAndWait();

    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                        // UPDATES OF STATE FOR PIDs //
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    void print_timer_callback();
    void update_timer_callback();

    void update_current_state();
    std::vector<float> getErrors(std::vector<float>& current_state, std::vector<float>& desired_state) ;
    std::vector<float> getThrusts(std::vector<float>& current_state, std::vector<float>& desired_state);
    std::vector<float> ctrlValsToThrusts(std::vector<float>& ctrl_vals);
    float angleBetweenHereAndPoint(float x, float y);
    float distanceBetweenHereAndPoint(float x, float y);
    std::vector<float> adjustErrors(std::vector<float>& errors);
    std::vector<float> update_PID(Interface::current_state_t& current_state, Interface::desired_state_t& desired_state);

    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                    // CAN REQUESTS FOR MOTOR CONTROL // 
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    std::vector<int> make_CAN_request(std::vector<float>& thrusts);

    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                            // ACTION SERVER // 
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const scion_types::action::PID::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<scion_types::action::PID>> goal_handle );
    void handle_accepted( const std::shared_ptr<rclcpp_action::ServerGoalHandle<scion_types::action::PID>> goal_handle);
    bool areEqual(float float1, float float2, float epsilon);
    bool areEqual(std::vector<float>& current_state, std::vector<float>& desired_state);
    bool equalToZero(std::vector<int> thrustVect);
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<scion_types::action::PID>> goal_handle);

    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                            // CONTROL SERVICES // 
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    void stopRobot(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void pidReady();
    void resetState();
    void resetPosition();
    void usePosition(const  std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void stabilizeRobot(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                        // SUBSCRIPTION CALLBACKS // 
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    
    void current_state_callback(const scion_types::msg::State::SharedPtr msg);

    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                        // GUI TUNING CONTROLS CALLBACKS // 
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    void kp_tuning_callback(const scion_types::msg::PidTuning::SharedPtr msg);
    void ki_tuning_callback(const scion_types::msg::PidTuning::SharedPtr msg);
    void kd_tuning_callback(const scion_types::msg::PidTuning::SharedPtr msg);
};

#endif