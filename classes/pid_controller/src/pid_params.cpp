#include "pid_params.hpp"

PID_Params::PID_Params()

{
    this->roll = 
    {
        {"kp", 0.3},
        {"ki", 0.14},
        {"kd", 0.02},
        {"ctrl_val_offset", 0.0},
        {"ctrl_val_max", 1.0},
        {"ctrl_val_min", -1.0},
        {"i_max", 1.0},
        {"i_min", -1.0}
    };

    this->pitch = 
    {
        {"kp", .2},
        {"ki", 0.11},
        {"kd", 0.07},
        {"ctrl_val_offset", 15.0},
        {"ctrl_val_max", 1.0},
        {"ctrl_val_min", -1.0},
        {"i_max", 1.0},
        {"i_min", -1.0}
    };


    this->yaw = 
    {
        {"kp", 0.3},
        {"ki", 0.05},
        {"kd", 0.01},
        {"ctrl_val_offset", 10.0},
        {"ctrl_val_max", 1.0},
        {"ctrl_val_min", -1.0},
        {"i_max", 1},
        {"i_min", -1}
    };
    
    this->x_pos =
    {
        {"kp", 0.2},
        {"ki", 0.13},
        {"kd", 0.06},
        {"ctrl_val_offset", 0.0},
        {"ctrl_val_max", 1.0},
        {"ctrl_val_min", -1.0},
        {"i_max", 1.0},
        {"i_min", -1.0}
    };


    this->y_pos = 
    {
        {"kp", 0.4},
        {"ki", 0.23},
        {"kd", 0.11},
        {"ctrl_val_offset", 16.0},
        {"ctrl_val_max", 1.0},
        {"ctrl_val_min", -1.0},
        {"i_max", 1.0},
        {"i_min", -1.0}
    };

    this->z_pos =
    {
        {"kp", 0.6},
        {"ki", 0.18},
        {"kd", 0.07},
        {"ctrl_val_offset", 20.0},
        {"ctrl_val_max", 1.0},
        {"ctrl_val_min", -1.0},
        {"i_max", 1.0},
        {"i_min", -1.0}
    };

    this->pid_params = 
    {
        {"roll",  this->roll},
        {"pitch", this->pitch},
        {"yaw",   this->yaw},
        {"x_pos", this->x_pos},
        {"y_pos", this->y_pos},
        {"z_pos", this->z_pos},
        {"x_vel", this->x_vel},
        {"y_vel", this->y_vel},
        {"z_vel", this->z_vel},
    };
}

map<string, map<string, double>> PID_Params::get_pid_params()
    {
        return this->pid_params;
    }


