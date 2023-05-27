/* 
 * @author Conner Sommerfield - Zix on Discord for questions
 * This is last year's PID code that I rewrote in C++ and adapted
 * The PID algorithm is implement in the pid_controller.cpp file
 * But we need a place to store info on each of those PID objects
 * That's what this file is for. It defines a Scion PID object
 * That keeps track of each PID (via shared pointers)
 */

#include "scion_pid_controller.hpp"


Scion_Position_PID_Controller::Scion_Position_PID_Controller()
{

 /*
  * Create 6 PID_Controller objects that will hold information about (roll, pitch, yaw, x, y, z).
  */

    // Angle wrapping set to true for roll-pitch-yaw PID controllers
    this->yaw_pid = std::make_shared<PID_Controller>(0.0, 0.0, 0.0, true);
    this->pitch_pid = std::make_shared<PID_Controller>(0.0, 0.0, 0.0, true);
    this->roll_pid = std::make_shared<PID_Controller>(0.0, 0.0, 0.0, true);

    this->x_pos_pid = std::make_shared<PID_Controller>(0.0, 0.0, 0.0);
    this->y_pos_pid = std::make_shared<PID_Controller>(0.0, 0.0, 0.0);
    this->z_pos_pid = std::make_shared<PID_Controller>(0.0, 0.0, 0.0);
    

    // Map strings to PID_Controller pointer to be able to refer to controller by name
    this->controllers = std::map<string, shared_ptr<PID_Controller>>
    (
        {
            {"yaw",   this->yaw_pid}, 
            {"pitch", this->pitch_pid},
            {"roll",  this->roll_pid},
            {"x_pos", this->x_pos_pid},
            {"y_pos", this->y_pos_pid},
            {"z_pos", this->z_pos_pid},
        }
    );

    // matrix mapping the 6 pid controller outputs to the 8 thrusters
    // -----yaw---pitch---roll---x---y---z
    //| T0
    //| T1
    //| T2
    //| T3
    //| T4
    //| T5
    //| T6
    //| T7
    

    // 6X1 matrix ctrls -> 8x1 matrix thrust -> Need 8x6 matrix * 6x1 matrix
    // 6X1 matrix -> 2X1 matrix, Need 2X6 matrix * 6x1 matrix

    this->pid_thrust_mapper = vector<vector<float>>
                                    {
                                        { 0,  1, -1,  0,  0,  1},                   
                                        { 1,  0,  0,  1,  1,  0},
                                        { 0, -1, -1,  0,  0,  1},
                                        { 1,  0,  0,  1, -1,  0},
                                        {-1, -1,  1,  0,  0,  1},
                                        { 0,  0,  0,  1,  1,  0},
                                        {-1,  1,  1,  0,  0,  1},
                                        { 0,  0,  0,  1, -1,  0}
                                    };
}

// PID params constructor will tune PID using pid_params.cpp values
Scion_Position_PID_Controller::Scion_Position_PID_Controller(map<string, map<string, float>> pid_params) : Scion_Position_PID_Controller::Scion_Position_PID_Controller()
{
    /* 
     * if we decide to load our PID_Params from a dictionary, we can set each controllers' values to 
     * the values stored in the hashmap from pid_params.cpp 
     */
    for (auto ctrl_type : this->controllers) // one row with string : unique_ptr<PID_Controller>
    {
        ctrl_type.second->k_p = pid_params[ctrl_type.first]["kp"];
        ctrl_type.second->k_i = pid_params[ctrl_type.first]["ki"];
        ctrl_type.second->k_d = pid_params[ctrl_type.first]["kd"];
        ctrl_type.second->ctrl_val_offset = pid_params[ctrl_type.first]["ctrl_val_offset"];
        ctrl_type.second->ctrl_val_max = pid_params[ctrl_type.first]["ctrl_val_max"];
        ctrl_type.second->ctrl_val_min = pid_params[ctrl_type.first]["ctrl_val_min"];
        ctrl_type.second->i_max = pid_params[ctrl_type.first]["i_max"];
        ctrl_type.second->i_min = pid_params[ctrl_type.first]["i_min"];
    }
}

     /* 
      * Every time we have new data on our current and desired position, we can tell each PID_Controller
      * to update their current state 
      */
vector<float> Scion_Position_PID_Controller::update
    (
        vector<float>& current_point,
        vector<float>& desired_point,
        float dt
    ) 
    {
        /**
        Perform PID controller update step and return the thrust to each of the 6 thrusters.
        
        :param set_point - The desired state of the vehicle [roll, pitch, yaw, x, y, z] 
        :param process_point - The current state of the vehicle [roll, pitch, yaw, x, y, z] 
        :param dt - Update interval in seconds (float)
        
        :return thrusts - A list of length 6 of the thrusts to apply to each motor: Range [-100, 100] 
        */
            
        pair<float, float> yaw_ctrl_data = this->yaw_pid->update(current_point[0], desired_point[0], dt);
        float yaw_ctrl_val = yaw_ctrl_data.first;
        // float yaw_error = yaw_ctrl_data.second;

        pair<float, float> pitch_ctrl_data = this->pitch_pid->update(current_point[1], desired_point[1], dt);
        float pitch_ctrl_val = pitch_ctrl_data.first;
        // float pitch_error = pitch_ctrl_data.second;

        pair<float, float> roll_ctrl_data = this->roll_pid->update(current_point[2], desired_point[2], dt);
        float roll_ctrl_val = roll_ctrl_data.first;
        // float roll_error = roll_ctrl_data.second;

        pair<float, float> x_pos_ctrl_data = this->x_pos_pid->update(current_point[3], desired_point[3], dt);
        float x_pos_ctrl_val = x_pos_ctrl_data.first;
        // float x_pos_error = x_pos_ctrl_data.second;

        pair<float, float> y_pos_ctrl_data = this->y_pos_pid->update(current_point[4], desired_point[4], dt);
        float y_pos_ctrl_val = y_pos_ctrl_data.first;
        // float y_pos_error = y_pos_ctrl_data.second;

        pair<float, float> z_pos_ctrl_data = this->z_pos_pid->update(current_point[5], desired_point[5], dt);
        float z_pos_ctrl_val = z_pos_ctrl_data.first;
        // float z_pos_error = z_pos_ctrl_data.second;

        // pull all ctrl_vals and errors into their respective vectors
        vector<float> ctrl_vals = vector<float>
        {yaw_ctrl_val, pitch_ctrl_val, roll_ctrl_val, x_pos_ctrl_val, y_pos_ctrl_val, z_pos_ctrl_val};
        // vector<float> pos_errors = vector<float>
        // {yaw_error, pitch_error, roll_error, x_pos_error, y_pos_error, z_pos_error};
        
        // map the individual controller outputs to each thruster.
        this->current_ctrl_vals = ctrl_vals;
        vector<float> thrusts = this->pid_thrust_mapper * ctrl_vals;
        this->current_thrust_values = thrusts;

        // pair<vector<float>, vector<float>> thrusts_and_errors{thrusts, pos_errors};
        return thrusts;
    }

// View the current state of all the PIDs on Scion and the latest ctrl_vals it has generated
void Scion_Position_PID_Controller::getStatus()
{
    cout << "REPORT FOR SCION PID" << endl;
    cout << "--------------------------------------------------------------------------------------" << endl;
    cout << endl;
    cout << "Scion PID Last Generated Control Values: ";
    printVector(this->current_ctrl_vals);
    cout << "Last Generated Thrust Values: ";
    printVector(this->current_thrust_values);

    for (auto ctrl_type : this->controllers)
    {      
        cout << endl;
        cout << "Printing Data For " << ctrl_type.first << "Controller:" << endl;
        ctrl_type.second->getStatus();
    }
}

//main for testing

// int main() 
// {
//     PID_Params pid_pirams_object;
//     map<string, map<string, float>> pid_params = pid_pirams_object.get_pid_params();

//     Scion_Position_PID_Controller controller = Scion_Position_PID_Controller();

//     // controller.getStatus();
//     /**
//      * Tune PID
//     */

//     controller.x_pos_pid->set_gains(.2,.13,.07);
//     controller.y_pos_pid->set_gains(.3,.11,.04); 
//     controller.z_pos_pid->set_gains(.15,.08,.02);  
//     controller.yaw_pid->set_gains(.2,.13,.07); 
//     controller.pitch_pid->set_gains(.3,.11,.04); 
//     controller.roll_pid->set_gains(.15,.08,.02);
    

//     controller.getStatus();


//     pair<vector<float>, vector<float>> answer = controller.update
//     (vector<float>{1.2, 2.4, 3.6, 4.8, 5.0, 6.1}, vector<float>{2.0, 3.0, 4.0, 5.0, 6.0, 7.0});
//     controller.getStatus();

//     vector<float> thrusts = answer.first;
//     vector<float> pos_errors = answer.second;

//     cout << "Thrusts: ";
//     printVector(thrusts);
//     cout << "Error_Values: ";
//     printVector(pos_errors);

//     answer = controller.update
//     (vector<float>{1.4, 2.6, 3.8, 4.9, 5.6, 6.6}, vector<float>{2.0, 3.0, 4.0, 5.0, 6.0, 7.0});
//     controller.getStatus();

//     thrusts = answer.first;
//     pos_errors = answer.second;

//     cout << "Thrusts: ";
//     printVector(thrusts);
//     cout << "Error_Values: ";
//     printVector(pos_errors);

//     answer = controller.update
//     (vector<float>{1.6, 2.8, 3.9, 5.0, 5.8, 6.8}, vector<float>{2.0, 3.0, 4.0, 5.0, 6.0, 7.0});
//     controller.getStatus();

//     thrusts = answer.first;
//     pos_errors = answer.second;

//     cout << "Thrusts: ";
//     printVector(thrusts);
//     cout << "Error_Values: ";
//     printVector(pos_errors);
    
//     return 0;
// }