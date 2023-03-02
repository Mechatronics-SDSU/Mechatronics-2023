/* 
 * @author Conner Sommerfield - Zix on Discord for questions
 * This PID algorithm was written before but I adapted it a little
 * bit for what we need and rewrote it in C++
 */

#include "pid_controller.hpp"

/** PID controller typically used in control feed back systems. */

PID_Controller::PID_Controller

    /**
     * Initialize the the PID controller parameters.
        Parameters:
            k_p: Proportional gain
            k_i: Integral gain
            k_d: Derivative gain
            i_min: minimum value the integral can accumulate
            i_max: maximum value the integral can accumulate
                Setting i_max < i_min will mean that no limits are set for integral term
            cmd_min: minimum output command
            cmd_max: maximum output command
                Setting cmd_max < cmd_min will mean no limits are set for the output cmd on update.
            cmd_offset: Offset to add to the command output on each update step.
            angle_wrap: if true, it assumes the input set point and process point are angles in range
            [-pi, pi]. Thus it will clamp the error in range [-pi, pi]
    */
   
    (
        float k_p, 
        float k_i, 
        float k_d, 
        bool angle_wrap,
        float i_min, 
        float i_max, 
        float ctrl_val_min, 
        float ctrl_val_max, 
        float ctrl_val_offset
    )

    {
        this->k_p = k_p;
        this->k_i = k_i;
        this->k_d = k_d;
        this->i_min = i_min;                        // Helps clamp integral overshoot
        this->i_max = i_max;
        this->ctrl_val_min = ctrl_val_min;          // Helps clamp output from -1 to 1
        this->ctrl_val_max = ctrl_val_max;
        this->ctrl_val_offset = ctrl_val_offset;
        this->angle_wrap = angle_wrap;
        this->integral = 0.0;                       // Keeps track of integral over time
        this->previous_error = 0.0;                 // Helps in derivative calculation
    }

void PID_Controller::set_gains(float k_p, float k_i, float k_d)
{
    /*
    Reset the gain parameters. These are constants that have to be tuned to fit the system.
    Parameters:
                    
        *  kp - kd - ki Explanation:

        *  kp (proportional gain) determines the proportion of the control signal that is proportional to the error
        *  between the set point and the process variable. The larger the value of kp, the stronger the controller 
        *  will respond to the error.
        * 
        *  ki (integral gain) determines the proportion of the control signal that is proportional to the integral
        *  of the error over time. This term helps to eliminate any residual error that may be present after 
        *  the proportional term has done its job 
        * 
        *  kd (derivative gain) determines the proportion of the control signal that is proportional to the rate 
        *  of change of the error. This term helps to reduce overshoot and oscillations in the control signal, 
        *  by predicting the future error based on the current rate of change.
    */

    this->k_p = k_p;
    this->k_i = k_i;
    this->k_d = k_d;
}


pair<float, float> PID_Controller::update(float current_point, float desired_point, float dt)
{
    /*
    Perfrom a control step to correct for error in control system.

    Parameters:
        current_point: The current state of the system
        desired_point: The desired state of the system
        dt: The interval between update steps.
    Returns:
        ctrl_val: A PID output control value to correct for error in system
    */


    /* compute error which is important in determining our proportional, integral, derivative */

    float error = desired_point - current_point;

    /* if error is for angular inputs (roll, pitch, yaw), perform angle wrapping. */
    if (this->angle_wrap)
    {
        if (error > 180)
        {
            error = error - (2 * 180);
        }
        else if (error < -180)
        {
            error = error + (2 * 180);
        }
    }
        
    /* Create our P-I-D based on error */
    float proportional = (this->k_p * error);  // Directly proportional to error based on k_p constant

    this->integral = this->integral + (error * dt);     // Integral builds over time
    if (this->i_min < this->i_max)                      // Clamp integral if necessary
    {
        if (this->integral < this->i_min)
        {
            this->integral = this->i_min;
        }
        else if (this->integral > this->i_max)
        {
            this->integral = this->i_max;
        }
    }
    float integral = this->k_i * this->integral;

    float derivative = this->k_d * (error - this->previous_error); // Derivative takes into account previous error

    this->previous_error = error;            // reset error for next cycle


    // std::cout << "proportional: " << proportional << endl;    //Just for testing
    // std::cout << "integral: " << integral << endl;
    // std::cout << "derivative: " << derivative << endl;

    /* Get our control value and clamp it if necessary */
    float pre_ctrl_val = proportional + integral + derivative;

    float ctrl_val;

    if (pre_ctrl_val >= 0)
    {
        ctrl_val = pre_ctrl_val + this->ctrl_val_offset;
    }
    else
    {
        ctrl_val = pre_ctrl_val - this->ctrl_val_offset;
    }

    if (this->ctrl_val_min < this->ctrl_val_max)
    {
        if(ctrl_val < this->ctrl_val_min)
        {
            ctrl_val = this->ctrl_val_min;
        }
        else if(ctrl_val > this->ctrl_val_max)
        {
            ctrl_val = this->ctrl_val_max;
        }
    }

    pair<float, float> ctrl_and_error(ctrl_val, error);

    return ctrl_and_error;
}

/* Print all PIDs fields */
void PID_Controller::getStatus()
{
    // cout << "k_p: " << this->k_p << endl;
    // cout << "k_i: " << this->k_i << endl;
    // cout << "k_d: " << this->k_d << endl;
    cout << "integral: " << this->integral << endl;
    cout << "previous_error: " << this->previous_error << endl;
}

// main for testing 

// int main()
// {
//     PID_Controller controller = PID_Controller(0.0F, 0.0F, 0.0F);

//     pair<float, float> ctrl_val_error = controller.update(5.0F, 3.0F, .5);

//     std::cout << ctrl_val_error.first;
//     std::cout << ctrl_val_error.second;

//     return 0;
// }