#ifndef PID_PARAMS_H
#define PID_PARAMS_H

#include <map>
#include <string>

using namespace std;

class PID_Params
{
    private:
        map<string, map<string, double>> pid_params;
        map<string, double> roll;
        map<string, double> pitch;
        map<string, double> yaw;
        map<string, double> x_pos;
        map<string, double> y_pos;
        map<string, double> z_pos;
        map<string, double> x_vel;
        map<string, double> y_vel;
        map<string, double> z_vel;

    public:
        PID_Params();

        map<string, map<string, double>> get_pid_params();

};

#endif