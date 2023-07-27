/* @author Zix
 * Organizes massive amount of declarations I need for the control system
 */

#include "control_interface.hpp"

Interface::matrix_t Interface::percy_thrust_mapper = Interface::matrix_t 
{
    { 0, -1, -1,  0,  0,  1},                   
    {-1,  0,  0, -1, -1,  0},
    { 0,  1, -1,  0,  0,  1},
    {-1,  0,  0, -1,  1,  0},
    { 0,  1,  1,  0,  0,  1},
    { 1,  0,  0, -1, -1,  0},
    { 0, -1,  1,  0,  0,  1},
    { 1,  0,  0, -1,  1,  0}
};

Interface::matrix_t Interface::junebug_thrust_mapper = Interface::matrix_t 
{
    {-1,  0,  0,  1,  0,  0},                   
    { 1,  0,  0,  1,  0,  0},
};


/* Defines Possible Commands to Be Given to the PID Controller */

Interface::desired_state_t Movements::turn(float degree)
{
    return desired_state_t{degree,0,0,0,0,0};
}

Interface::desired_state_t Movements::pitch(float degree)
{
    return desired_state_t{0,degree,0,0,0,0};
}

Interface::desired_state_t Movements::roll(float degree)
{
    return desired_state_t{0,0,degree,0,0,0};
}

Interface::desired_state_t Movements::move(float degree)
{
    return desired_state_t{0,0,0,degree,0,0};
}

Interface::desired_state_t Movements::translate(float degree)
{
    return desired_state_t{0,0,0,0,degree,0};
}

Interface::desired_state_t Movements::levitate(float degree)
{
    return desired_state_t{0,0,0,0,0,degree};
}


/* All Translator Functions take an idea and translate it into a series of commands to add to mediator queue */

Interface::command_vector_t Translator::turn(float degree)
{
    Interface::Command command1;
    command1.function.transform = &Movements::turn;
    command1.params.degree = degree;
    return command_vector_t{command1};
}

Interface::command_vector_t Translator::roll(float degree)
{
    Interface::Command command1;
    command1.function.transform = &Movements::roll;
    command1.params.degree = degree;
    return command_vector_t{command1};
}

Interface::command_vector_t Translator::pitch(float degree)
{
    Interface::Command command1;
    command1.function.transform = &Movements::pitch;
    command1.params.degree = degree;
    return command_vector_t{command1};
}

Interface::command_vector_t Translator::move(float degree)
{
    Interface::Command command1;
    command1.function.transform = &Movements::move;
    command1.params.degree = degree;
    return command_vector_t{command1};
}

Interface::command_vector_t Translator::translate(float degree)
{
    Interface::Command command1;
    command1.function.transform = &Movements::translate;
    command1.params.degree = degree;
    return command_vector_t{command1};
}

Interface::command_vector_t Translator::levitate(float degree)
{
    Interface::Command command1;
    command1.function.transform = &Movements::levitate;
    command1.params.degree = degree;
    return command_vector_t{command1};
}

Interface::command_vector_t Translator::relativePoint(float x, float y)
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

Interface::command_vector_t Translator::absolutePoint(float x, float y)
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

Interface::command_vector_t Translator::pureRelativePoint(float x, float y)
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

Interface::command_vector_t Translator::pureAbsolutePoint(float x, float y)
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

void canClient::sendFrame(int32_t can_id, int8_t can_dlc, unsigned char can_data[], Interface::ros_sendframe_client_t can_client)
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

void canClient::setBotInSafeMode(Interface::ros_sendframe_client_t can_client)
{
    std::vector<unsigned char> safeModeFrame{0,0,0,0,0x04};
    sendFrame(0x022, 5, safeModeFrame.data(), can_client);
}


