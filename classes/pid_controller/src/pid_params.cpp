/*
 * @author Conner Sommerfield - Zix on Discord for questions
 * Took json file written by Tristan and translated to C++ plus
 * changed some things.
 * These are PID parameters to tune the PID on initialization
 * Read about k_p, k_i, and k_d in the pid_controller.cpp file
 * Use MAX and MIN values to clamp our values to a reasonable range
 */


#include "pid_params.hpp"
#define MAX .5
#define MIN -.5

/* 
    To make sure motor isn't running at maximum value, we want to cap our ctrl_val min/max. You can do this changing the macros up above to 
    your desired cap value.

    k_p value should be set to a very low value for orientation to keep from always blasting the motors
*/

PID_Params::PID_Params()


{
    this->yaw = 
    {
        {"kp", 0.0015},  // .015
        {"ki", 0.0003},   // .003
        {"kd", 0.0006},  // .006
        // {"ctrl_val_offset", 10.0},
        {"ctrl_val_max", MAX},
        {"ctrl_val_min", MIN},
        {"i_max", MAX},
        {"i_min", MIN}
    };

    this->pitch = 
    {
        {"kp", 0.0015},
        {"ki", 0.0003},
        {"kd", 0.0006},
        {"ctrl_val_offset", 0.0},
        {"ctrl_val_max", MAX},
        {"ctrl_val_min", MIN},
        {"i_max", MAX},
        {"i_min", MIN}
    };
    
    this->roll = 
    {
        {"kp", 0.0015},
        {"ki", 0.0003},
        {"kd", 0.0006},
        // {"ctrl_val_offset", 0.0},
        {"ctrl_val_max", MAX},
        {"ctrl_val_min", MIN},
        {"i_max", MAX},
        {"i_min", MIN}
    };
    
    this->x_pos =
    {
        {"kp", 0.016},
        {"ki", 0.005},
        {"kd", 0.008},
        {"ctrl_val_offset", 0.0},
        {"ctrl_val_max", MAX},
        {"ctrl_val_min", MIN},
        {"i_max", MAX},
        {"i_min", MIN}
    };

    this->y_pos = 
    {
        {"kp", 0.016},
        {"ki", 0.005},
        {"kd", 0.008},
        {"ctrl_val_offset", 0.0},
        {"ctrl_val_max", MAX},
        {"ctrl_val_min", MIN},
        {"i_max", MAX},
        {"i_min", MIN}
    };

    this->z_pos =
    {
        {"kp", 0.800},
        {"ki", 0.003},
        {"kd", 0.002},
        {"ctrl_val_offset", 0.0},
        {"ctrl_val_max", MAX},
        {"ctrl_val_min", MIN},
        {"i_max", MAX},
        {"i_min", MIN}
    };

    // this->yaw = 
    // {
    //     {"kp", 0.012},  // .015
    //     {"ki", 0.003},   // .003
    //     {"kd", 0.006},  // .006
    //     {"ctrl_val_offset", 0.0},
    //     {"ctrl_val_max", MAX},
    //     {"ctrl_val_min", MIN},
    //     {"i_max", MAX},
    //     {"i_min", MIN}
    // };

    // this->pitch = 
    // {
    //     {"kp", 0.00015},
    //     {"ki", 0.00003},
    //     {"kd", 0.00006},
    //     {"ctrl_val_offset", 0.0},
    //     {"ctrl_val_max", MAX},
    //     {"ctrl_val_min", MIN},
    //     {"i_max", MAX},
    //     {"i_min", MIN}
    // };
    
    // this->roll = 
    // {
    //     {"kp", 0.00015},
    //     {"ki", 0.00003},
    //     {"kd", 0.00006},
    //     {"ctrl_val_offset", 0.0},
    //     {"ctrl_val_max", MAX},
    //     {"ctrl_val_min", MIN},
    //     {"i_max", MAX},
    //     {"i_min", MIN}
    // };
    
    // this->x_pos =
    // {
    //     {"kp", 0.4},
    //     {"ki", 0.03},
    //     {"kd", 0.01},
    //     {"ctrl_val_offset", 0.0},
    //     {"ctrl_val_max", MAX},
    //     {"ctrl_val_min", MIN},
    //     {"i_max", MAX},
    //     {"i_min", MIN}
    // };

    // this->y_pos = 
    // {
    //     {"kp", 0.4},
    //     {"ki", 0.03},
    //     {"kd", 0.01},
    //     {"ctrl_val_offset", 0.0},
    //     {"ctrl_val_max", MAX},
    //     {"ctrl_val_min", MIN},
    //     {"i_max", MAX},
    //     {"i_min", MIN}
    // };

    // this->z_pos =
    // {
    //     {"kp", 0.00001},
    //     {"ki", 0.00001},
    //     {"kd", 0.00001},
    //     {"ctrl_val_offset", 0.0},
    //     {"ctrl_val_max", MAX},
    //     {"ctrl_val_min", MIN},
    //     {"i_max", MAX},
    //     {"i_min", MIN}
    // };

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

map<string, map<string, float>> PID_Params::get_pid_params()
    {
        return this->pid_params;
    }


