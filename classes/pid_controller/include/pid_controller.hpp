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
    private:

    public:
        double k_p;
        double k_i;
        double k_d;
        double i_min;
        double i_max;
        double ctrl_val_min;
        double ctrl_val_max;
        double ctrl_val_offset;
        bool angle_wrap;
        double integral = 0.0;
        double previous_error = 0.0;
        
        PID_Controller() {};

        PID_Controller
        (
            double k_p, 
            double k_i, 
            double k_d, 
            bool angle_wrap=false,
            double i_min=-1.0, 
            double i_max=1.0, 
            double ctrl_val_min=-1.0, 
            double ctrl_val_max=1.0, 
            double ctrl_val_offset=0.0
        );
   
        void set_gains(double k_p, double k_i, double k_d);

        pair<double, double> update(double set_point, double process_point, double dt);

        void getStatus();
};

#endif /* PID_CONTROLLER_H */