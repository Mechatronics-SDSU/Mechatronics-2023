#ifndef PID_PARAMS_H
#define PID_PARAMS_H

#include <map>
#include <string>

using namespace std;

class PID_Params
{
    private:
        map<string, map<string, float>> pid_params;
        map<string, float> roll;
        map<string, float> pitch;
        map<string, float> yaw;
        map<string, float> x_pos;
        map<string, float> y_pos;
        map<string, float> z_pos;
        map<string, float> x_vel;
        map<string, float> y_vel;
        map<string, float> z_vel;

    public:
        PID_Params();

        map<string, map<string, float>> get_pid_params();

};

#endif