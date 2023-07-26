/* @author Zix
 * Organizes massive amount of declarations I need for the control system
 */

#ifndef CONTROL_INTERFACE_H
#define CONTROL_INTERFACE_H

#include <vector>
#include <iostream>
#include <deque>
#include <unordered_map>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "scion_types/srv/send_frame.hpp"
#include "scion_types/msg/idea.hpp"
#include "scion_types/msg/state.hpp"
#include "scion_types/msg/position.hpp"
#include "scion_types/msg/orientation.hpp"
#include "scion_types/msg/datapoint.hpp"
#include "scion_types/msg/vision_object.hpp"
#include "scion_types/msg/zed_object.hpp"
#include "scion_types/msg/pid_tuning.hpp"
#include "scion_types/action/pid.hpp"
#include "std_msgs/msg/int32.hpp"


#define PI 3.14159265

using PIDAction = scion_types::action::PID;

namespace Interface
{
    union Function;      
    union Params; 
    struct Command;

    /* Custom Types to Use In Control System */
    typedef std::vector<float>                                                              current_state_t;
    typedef std::vector<float>                                                              desired_state_t;
    typedef scion_types::msg::Idea                                                          idea_message_t;
    typedef std::vector<scion_types::msg::Idea>                                             idea_vector_t;
    typedef std::deque<Command>                                                             command_queue_t;
    typedef std::vector<Command>                                                            command_vector_t;

    typedef rclcpp::Node::SharedPtr                                                         node_t;
    typedef rclcpp_action::Client<PIDAction>::SharedPtr                                     pid_action_client_t;
    typedef rclcpp_action::Server<PIDAction>::SharedPtr                                     pid_action_server_t;
    typedef rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr                               ros_trigger_client_t;
    typedef rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr                               ros_bool_client_t;
    typedef rclcpp::Client<scion_types::srv::SendFrame>::SharedPtr                          ros_sendframe_client_t;
    typedef rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                              ros_trigger_service_t;
    typedef rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr                              ros_bool_service_t;
    typedef rclcpp::Service<scion_types::srv::SendFrame>::SharedPtr                         ros_sendframe_service_t;
    typedef rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr                           int_sub_t;
    typedef rclcpp::Subscription<scion_types::msg::PidTuning>::SharedPtr                    tune_pid_sub_t;
    typedef rclcpp::Subscription<scion_types::msg::Idea>::SharedPtr                         idea_sub_t;
    typedef rclcpp::Subscription<scion_types::msg::State>::SharedPtr                        state_sub_t;
    typedef rclcpp::Subscription<scion_types::msg::Position>::SharedPtr                     position_sub_t;
    typedef rclcpp::Subscription<scion_types::msg::Orientation>::SharedPtr                  orientation_sub_t;
    typedef rclcpp::Subscription<scion_types::msg::Datapoint>::SharedPtr                    datapoint_sub_t;
    typedef rclcpp::Subscription<scion_types::msg::VisionObject>::SharedPtr                 object_sub_t;
    typedef rclcpp::Subscription<scion_types::msg::ZedObject>::SharedPtr                    vision_sub_t;
    typedef rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr                              int_pub_t;
    typedef rclcpp::Publisher<scion_types::msg::PidTuning>::SharedPtr                       tune_pid_pub_t;
    typedef rclcpp::Publisher<scion_types::msg::Idea>::SharedPtr                            idea_pub_t;
    typedef rclcpp::Publisher<scion_types::msg::State>::SharedPtr                           state_pub_t;
    typedef rclcpp::Publisher<scion_types::msg::Position>::SharedPtr                        position_pub_t;
    typedef rclcpp::Publisher<scion_types::msg::Orientation>::SharedPtr                     orientation_pub_t;
    typedef rclcpp::Publisher<scion_types::msg::Datapoint>::SharedPtr                       datapoint_pub_t;
    typedef rclcpp::Publisher<scion_types::msg::VisionObject>::SharedPtr                    object_pub_t;
    typedef rclcpp::Publisher<scion_types::msg::ZedObject>::SharedPtr                       vision_pub_t;
    typedef std::vector<std::vector<float>>                                                 matrix_t;                                                                            
    typedef rclcpp::TimerBase::SharedPtr                                                    ros_timer_t;


    /* Function Pointers That Point to Different Commands for Robot */
    typedef desired_state_t (*state_transform_func)(float); //, current_state_t&
    typedef desired_state_t (*simple_movement_func)();
    typedef bool (*predicate_function)();

    /* Brain will Send Ideas to the Mediator Which will Translate Into Commands For the PID Controller */
    enum Idea
    {
        STOP =     0,
        GO =       1,
        SPIN =     2,
        TURN =          3,
        PITCH =         4,
        ROLL =          5,
        MOVE =              6,
        TRANSLATE =         7,
        LEVITATE =          8,
        RELATIVE_POINT =        9,
        ABSOLUTE_POINT =        10,
        PURE_RELATIVE_POINT =   11,
        PURE_ABSOLUTE_POINT =   12
    };

    /*
     * Function Pointers May Point to Functions with Different Arguments. To not Have to Make Different
     * Structs Based on this, we can use a union and pick the signature that specific command will require
     * All function pointers conveniently have the same size so it will be perfect in the union.
     */
    union Function
    {
        simple_movement_func movement;
        state_transform_func transform;
    };      

    union Params
    {
        float degree;
        std::vector<float>* listPtr;
        void* nothing = nullptr;
    }; 

    struct Command
    {
        Function function;           // Points to a function to execute          (turn)
        Params params;               // The magnitude to pass into that function (30 degrees)
    };

    extern std::vector<std::vector<float>> percy_thrust_mapper;

    extern std::vector<std::vector<float>> junebug_thrust_mapper;
}

/* Defines Possible Commands to Be Given to the PID Controller */
namespace Movements
{
    using namespace Interface;

    desired_state_t turn(float degree);

    desired_state_t pitch(float degree);

    desired_state_t roll(float degree);

    desired_state_t move(float degree);

    desired_state_t translate(float degree);

    desired_state_t levitate(float degree);
}

/* All Translator Functions take an idea and translate it into a series of commands to add to mediator queue */
namespace Translator
{
    using namespace Interface;
    
    command_vector_t turn(float degree);

    command_vector_t roll(float degree);

    command_vector_t pitch(float degree);

    command_vector_t move(float degree);

    command_vector_t translate(float degree);

    command_vector_t levitate(float degree);

    command_vector_t relativePoint(float x, float y);

    command_vector_t absolutePoint(float x, float y);

    command_vector_t pureRelativePoint(float x, float y);

    command_vector_t pureAbsolutePoint(float x, float y);
}

namespace canClient
{
    void sendFrame(int32_t can_id, int8_t can_dlc, unsigned char can_data[], Interface::ros_sendframe_client_t can_client);
    void setBotInSafeMode(Interface::ros_sendframe_client_t can_client);
}

#endif