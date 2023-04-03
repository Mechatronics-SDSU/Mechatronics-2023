
#include <vector>
#include <iostream>
#include <deque>
#include <unordered_map>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "scion_types/msg/idea.hpp"

#define PI 3.14159265

namespace Interface
{
    union Function;      
    union Params; 
    struct Command;

    /* Custom Types to Use In Control System */
    typedef std::vector<float>                                  current_state_t;
    typedef std::vector<float>                                  desired_state_t;
    typedef scion_types::msg::Idea                              idea_message_t;
    typedef std::vector<scion_types::msg::Idea>                 idea_vector_t;
    typedef std::deque<Command>                                 command_queue_t;
    typedef std::vector<Command>                                command_vector_t;

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
        RELATIVE_POINT = 7,
        ABSOLUTE_POINT = 8,
        PURE_RELATIVE_POINT = 9,
        PURE_ABSOLUTE_POINT = 10

        // YAW = 4,
        // PITCH = 5,
        // ROLL = 6,
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
        std::cout << "Turn " << degree << " New " << std::endl;
        return desired_state_t{degree,0,0,0,0,0};
    }

    desired_state_t move(float degree)
    {
        std::cout << "Move " << degree << std::endl;
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
        std::cout << "Turn" << degree << std::endl;
        return command_vector_t{};
    }

    command_vector_t move(float degree)
    {
        std::cout << "Move" << degree << std::endl;
        return command_vector_t{};
    }

    command_vector_t count()
    {
        return command_vector_t{};
    }

    command_vector_t relativePoint(float x, float y)
    {
        float point_angle_radians = atan(x / y);

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

    command_vector_t absolutePoint(float x, float y)
    {
        float point_angle_radians = atan(y / x);
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