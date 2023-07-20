#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#define _USE_MATH_DEFINES           // This is to get math constants like PI
#include <utility>
#include <vector>
#include <cmath>
#include <iostream>

using namespace std;

/** PID controller typically used in control feed back systems. */

class PID_Controller
{
    public:
        float k_p;
        float k_i;
        float k_d;
        float i_min;
        float i_max;
        float ctrl_val_min;
        float ctrl_val_max;
        float ctrl_val_offset;
        bool angle_wrap;
        float integral = 0.0;
        float previous_error = 0.0;
        float curr_ctrl_val = 0.0;
        float enabled = true;
        
        PID_Controller() {};

        PID_Controller
        (
            float k_p, 
            float k_i, 
            float k_d, 
            bool angle_wrap=false,
            float i_min=-1.0, 
            float i_max=1.0, 
            float ctrl_val_min=-1.0, 
            float ctrl_val_max=1.0, 
            float ctrl_val_offset=0.0
        );
   
        void set_gains(float k_p, float k_i, float k_d);

        float update(float error, float dt);

        void disable();

        void enable();

        void getStatus();
};

#endif /* PID_CONTROLLER_H */