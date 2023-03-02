#ifndef SCION_PID_CONTROLLER_H
#define SCION_PID_CONTROLLER_H

#include <map>
#include <string>
#include <utility>
#include <tuple>
#include <memory>

#include "pid_controller.hpp"
#include "pid_params.hpp"
#include "vector_operations.hpp" // matrix multiplication

using namespace std;

/** 
  * Create a "positional" pid control system for the Scion. This system constains a positional pid layer
  * that directly feeds into a velocity PID layer. There will be 6 directional controller for
  * (roll, pitch, yaw, x, y, z). These are the DOFs controllable by the actuator.
*/

class Scion_Position_PID_Controller
{
    private:

    public:
        shared_ptr<PID_Controller> yaw_pid;
        shared_ptr<PID_Controller> pitch_pid;
        shared_ptr<PID_Controller> roll_pid;
        shared_ptr<PID_Controller> x_pos_pid;
        shared_ptr<PID_Controller> y_pos_pid;
        shared_ptr<PID_Controller> z_pos_pid;
        std::map<string, shared_ptr<PID_Controller>> controllers;
        vector<float> current_ctrl_vals;
        vector<float> current_thrust_values;
        vector<vector<float>> pid_thrust_mapper;

        // Default constructor to tune PID manually
        Scion_Position_PID_Controller();

        // Or put tuning values into pid_params.cpp and call from here
        Scion_Position_PID_Controller(map<string, map<string, float>> pid_params);
    
        // Request control values from each PID
        pair<vector<float>, vector<float>> update
            (
                vector<float> current_point=    vector<float>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                vector<float> desired_point=vector<float>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 
                float dt=0.010
            );

        // Print status of all PIDs to console
        void getStatus();

};

#endif /* SCION_PID_CONTROLLER */