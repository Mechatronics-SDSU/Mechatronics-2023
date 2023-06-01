/* @author Zix
 * Organizes massive amount of declarations I need for the control system
 */


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
#include "scion_types/action/pid.hpp"


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

    typedef rclcpp_action::Client<PIDAction>::SharedPtr                                     pid_action_client_t;
    typedef rclcpp_action::Server<PIDAction>::SharedPtr                                     pid_action_server_t;
    typedef rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr                               ros_trigger_client_t;
    typedef rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr                               ros_bool_client_t;
    typedef rclcpp::Client<scion_types::srv::SendFrame>::SharedPtr                          ros_sendframe_client_t;
    typedef rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                              ros_trigger_service_t;
    typedef rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr                              ros_bool_service_t;
    typedef rclcpp::Service<scion_types::srv::SendFrame>::SharedPtr                         ros_sendframe_service_t;
    typedef rclcpp::Subscription<scion_types::msg::Idea>::SharedPtr                         idea_sub_t;
    typedef rclcpp::Subscription<scion_types::msg::State>::SharedPtr                        state_sub_t;
    typedef rclcpp::Subscription<scion_types::msg::Position>::SharedPtr                     position_sub_t;
    typedef rclcpp::Subscription<scion_types::msg::Orientation>::SharedPtr                  orientation_sub_t;
    typedef rclcpp::Publisher<scion_types::msg::Idea>::SharedPtr                            idea_pub_t;
    typedef rclcpp::Publisher<scion_types::msg::State>::SharedPtr                           state_pub_t;
    typedef rclcpp::Publisher<scion_types::msg::Position>::SharedPtr                        position_pub_t;
    typedef rclcpp::Publisher<scion_types::msg::Orientation>::SharedPtr                     orientation_pub_t;
    typedef rclcpp::TimerBase::SharedPtr                                                    ros_timer_t;



    /* Function Pointers That Point to Different Commands for Robot */
    typedef desired_state_t (*state_transform_func)(float); //, current_state_t&
    typedef desired_state_t (*simple_movement_func)();

    /* Brain will Send Ideas to the Mediator Which will Translate Into Commands For the PID Controller */
    enum Idea
    {
        STOP = 0,
        GO = 1,
        SPIN = 2,
        MOVE = 3,
        TURN = 4,
        // PITCH = 5,
        // ROLL = 6,
        RELATIVE_POINT = 7,
        ABSOLUTE_POINT = 8,
        PURE_RELATIVE_POINT = 9,
        PURE_ABSOLUTE_POINT = 10
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
}

/* Defines Possible Commands to Be Given to the PID Controller */
namespace Movements
{
    using namespace Interface;

    desired_state_t stop()
    {
        std::cout << "Stop " << std::endl;
        return desired_state_t{0,0,0,0,0,0};
    }

    desired_state_t spin(float seconds)
    {
        std::cout << "Turn " << std::endl;
        return desired_state_t{0,0,0,0,0,0};
    }

    desired_state_t pitch(float degree)
    {
        std::cout << "Pitch " << std::endl;
        return desired_state_t{0,0,0,0,0,0};
    }

    desired_state_t go(float seconds)
    {
        std::cout << "Move " << std::endl;
        return desired_state_t{10,10,10,10,10,10};
    }

    desired_state_t turn(float degree)
    {
        return desired_state_t{degree,0,0,0,0,0};
    }

    desired_state_t move(float degree)
    {
        return desired_state_t{0,0,0,degree,0,0};
    }
}

/* All Translator Functions take an idea and translate it into a series of commands to add to mediator queue */
namespace Translator
{
    using namespace Interface;
    
    command_vector_t stop()
    {
        std::cout << "Stop" << std::endl;
        return command_vector_t{};
    }

    command_vector_t spin(float seconds)
    {
        std::cout << "Turn" << std::endl;
        return command_vector_t{};
    }

    command_vector_t pitch(float degree)
    {
        std::cout << "Pitch" << std::endl;
        return command_vector_t{};
    }

    command_vector_t go(float seconds)
    {
        std::cout << "Move" << std::endl;
        return command_vector_t{};
    }

    command_vector_t turn(float degree)
    {
        Interface::Command command1;
        command1.function.transform = &Movements::turn;
        command1.params.degree = degree;
        return command_vector_t{command1};
    }

    command_vector_t move(float degree)
    {
        Interface::Command command1;
        command1.function.transform = &Movements::move;
        command1.params.degree = degree;
        return command_vector_t{command1};
    }

    command_vector_t count()
    {
        return command_vector_t{};
    }

    command_vector_t relativePoint(float x, float y)
    {
        float point_angle_radians = atan(x / y);
        float point_angle_degrees = point_angle_radians * (180/PI);
        if (y < 0)
        {
            point_angle_degrees += 180;
        }
        float point_distance_meters = sqrt(pow(x,2) + pow(y,2));

        Interface::Command command1;
        command1.function.transform = &Movements::turn;
        command1.params.degree = point_angle_degrees;

        Interface::Command command2;
        command2.function.transform = &Movements::move;
        command2.params.degree = point_distance_meters;

        return command_vector_t{command1, command2};
    }

    command_vector_t absolutePoint(float x, float y)
    {
        float point_angle_radians = atan(y / x);
        float point_angle_degrees = point_angle_radians * (180/PI);
        float point_distance_meters = sqrt(pow(x,2) + pow(y,2));

        Interface::Command command1;
        command1.function.transform = &Movements::turn;
        command1.params.degree = point_angle_degrees;

        Interface::Command command2;
        command2.function.transform = &Movements::move;
        command2.params.degree = point_distance_meters;

        return command_vector_t{command1, command2};
    }

    command_vector_t pureRelativePoint(float x, float y)
    {
        float point_angle_radians = atan(y / x);
        float point_angle_degrees = point_angle_radians * (180/PI);
        float point_distance_meters = sqrt(pow(x,2) + pow(y,2));

        Interface::Command command1;
        command1.function.transform = &Movements::turn;
        command1.params.degree = point_angle_degrees;

        Interface::Command command2;
        command2.function.transform = &Movements::move;
        command2.params.degree = point_distance_meters;

        return command_vector_t{command1, command2};
    }

    command_vector_t pureAbsolutePoint(float x, float y)
    {
        float point_angle_radians = atan(y / x);
        float point_angle_degrees = point_angle_radians * (180/PI);
        float point_distance_meters = sqrt(pow(x,2) + pow(y,2));

        Interface::Command command1;
        command1.function.transform = &Movements::turn;
        command1.params.degree = point_angle_degrees;

        Interface::Command command2;
        command2.function.transform = &Movements::move;
        command2.params.degree = point_distance_meters;

        return command_vector_t{command1, command2};
    }
}

namespace canClient
{
    void sendFrame(int32_t can_id, int8_t can_dlc, unsigned char can_data[], Interface::ros_sendframe_client_t can_client)
    {
      auto can_request = std::make_shared<scion_types::srv::SendFrame::Request>();
      can_request->can_id = can_id;
      can_request->can_dlc = can_dlc;
      std::copy
      (
          can_data,
          can_data + can_dlc,
          can_request->can_data.begin()
      );
      auto can_future = can_client->async_send_request(can_request);
    }

    void setBotInSafeMode(Interface::ros_sendframe_client_t can_client)
    {
        vector<unsigned char> safeModeFrame{0,0,0,0,0x04};
        sendFrame(0x022, 5, safeModeFrame.data(), can_client);
    }
}