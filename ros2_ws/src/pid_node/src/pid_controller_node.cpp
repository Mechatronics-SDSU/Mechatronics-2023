/* 
 * @author Conner Sommerfield - Zix on Discord
 * The infamous PID Controller Node
 * Subscribes to all sensor data needed for control system and throws it into a PID object
 * to output ctrl and thrust values.
 * Refer to classes/pid_controller for more info
 * 
 * Every time the subscription callback is triggered, the current state will be updated
 * Current state is stored in member variables of the Node
 * There is a timer that updates every x amount of ms based on what user inputs
 * This calls the PID update function which will use that sensor data to get ctrl/thrust values
 * Then it makes a CAN Request using that array of thrust_vals to send to motors
 * CAN Request is of ID #010
 */

#include <memory>
#include <string>
#include <ctime>
#include <thread>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <vector>
#include <cstring>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_interface.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "scion_types/action/pid.hpp"
#include "scion_types/msg/state.hpp"
#include "scion_pid_controller.hpp"                // PID Class
#include "pid_controller.hpp"   
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;

#define SLEEP_TIME 500
#define UPDATE_PERIOD 100ms
#define UPDATE_PERIOD_RAW 10
#define PRINT_PERIOD 500ms
#define PID_ERROR_THRESHOLD 0.01f
#define MOTOR_ID 0x010

/////////////////////////////////////////////////////////////////////////////////////////////////////
                                    // MEMBER VARIABLE DECLARATIONS // 
/////////////////////////////////////////////////////////////////////////////////////////////////////

using PIDAction = scion_types::action::PID;
using GoalHandlePIDAction = rclcpp_action::ServerGoalHandle<PIDAction>;

/** 
 * Controller Node consists of: 
 *      - timer to update ctrl_vals
 *      - subscription to desired state
 *      - subscription to ms5837 depth data
 *      - subscription to ZED position data
 *      - subscription to AHRS orientation data
 *      - Creates CAN Mailbox and ScionPIDController objects 
**/
class Controller : public rclcpp::Node
{
public:
    explicit Controller(): Node("pid_controller")
    {

        this->declare_parameter("thrust_mapper", "percy");

        string thrust_mapper = this->get_parameter("thrust_mapper").as_string();

        if (thrust_mapper == "percy")
        {
            this->thrust_mapper_ = Interface::percy_thrust_mapper;
        } 
        else if (thrust_mapper == "junebug")
        {
            this->thrust_mapper_ = Interface::junebug_thrust_mapper;
            this->motor_count_ = 2;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "GIVE VALID VALUE FOR THRUST MAPPER IN LAUNCH FILE");
            exit(EXIT_FAILURE);
        }

        update_timer_ = this->create_wall_timer(UPDATE_PERIOD, std::bind(&Controller::update_timer_callback, this));
        
        print_timer_ = this->create_wall_timer(PRINT_PERIOD, std::bind(&Controller::print_timer_callback, this));

        current_state_sub_ = this->create_subscription<scion_types::msg::State>
        ("relative_current_state_data", 10, std::bind(&Controller::current_state_callback, this, _1));

        axis_tuning_map_ = std::map<int, string> 
        ({
            {0, "yaw"  }, 
            {1, "pitch"},
            {2, "roll" },
            {3, "x_pos"},
            {4, "y_pos"},
            {5, "z_pos"},
        });

        kp_tuning_sub_ = this->create_subscription<scion_types::msg::PidTuning>("kp_dial_data", 10, std::bind(&Controller::kp_tuning_callback, this, _1));
        ki_tuning_sub_ = this->create_subscription<scion_types::msg::PidTuning>("ki_dial_data", 10, std::bind(&Controller::ki_tuning_callback, this, _1));
        kd_tuning_sub_ = this->create_subscription<scion_types::msg::PidTuning>("kd_dial_data", 10, std::bind(&Controller::kd_tuning_callback, this, _1));

        // desired_state_sub_ = this->create_subscription<scion_types::msg::State>
        // ("desired_state_data", 10, std::bind(&Controller::desired_state_callback, this, _1));

        this->pid_command_server_ = rclcpp_action::create_server<PIDAction>
        (
            this,
            "PIDAction",
            std::bind(&Controller::handle_goal, this, _1, _2),
            std::bind(&Controller::handle_cancel, this, _1),
            std::bind(&Controller::handle_accepted, this, _1)
        );

        can_client_ = this->create_client<scion_types::srv::SendFrame>("send_can_raw");
        reset_relative_state_client_ = this->create_client<std_srvs::srv::Trigger>("reset_relative_state");
        reset_relative_position_client_ = this->create_client<std_srvs::srv::Trigger>("reset_relative_position");
        pid_ready_client_ = this->create_client<std_srvs::srv::Trigger>("pid_ready");
        stop_robot_service_ = this->create_service<std_srvs::srv::Trigger>("stop_robot", std::bind(&Controller::stopRobot, this, _1, _2));
        use_position_service_ = this->create_service<std_srvs::srv::SetBool>("use_position", std::bind(&Controller::usePosition, this, _1, _2));
        stabilize_robot_service_ = this->create_service<std_srvs::srv::SetBool>("stabilize_robot", std::bind(&Controller::stabilizeRobot, this, _1, _2));

        controller_ = Scion_Position_PID_Controller(pid_params_object_.get_pid_params());
        controller_.getStatus();

        canClient::setBotInSafeMode(can_client_);

        auto initFunction = std::bind(&Controller::initDesiredState, this);
        std::thread(initFunction).detach();
    }

private:
    Interface::ros_timer_t                      update_timer_;
    Interface::ros_timer_t                      print_timer_;
    Interface::state_sub_t                      current_state_sub_;
    Interface::state_sub_t                      desired_state_sub_;
    Interface::tune_pid_sub_t                   kp_tuning_sub_;
    Interface::tune_pid_sub_t                   ki_tuning_sub_;
    Interface::tune_pid_sub_t                   kd_tuning_sub_;
    Interface::pid_action_server_t              pid_command_server_;
    Interface::ros_bool_service_t               use_position_service_;
    Interface::ros_bool_service_t               stabilize_robot_service_;
    Interface::ros_trigger_service_t            stop_robot_service_;
    Interface::ros_trigger_client_t             reset_relative_state_client_;        
    Interface::ros_trigger_client_t             reset_relative_position_client_;
    Interface::ros_sendframe_client_t           can_client_;
    Interface::matrix_t                         thrust_mapper_;
    Scion_Position_PID_Controller               controller_;
    PID_Params                                  pid_params_object_;                      // Passed to controller for tuning
    Interface::ros_trigger_client_t             pid_ready_client_;
    int                                         motor_count_ = 8;
    std::map<int, string>                       axis_tuning_map_;

    /* Upon initialization set all values to [0,0,0] */
    
    Interface::current_state_t current_state_{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F}; // State described by yaw, pitch, roll, x, y, z 
    Interface::desired_state_t desired_state_{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F}; // Desired state is that everything is set to 0 except that its 1 meter below the water {0,0,0,0,0,1}
    vector<unsigned char> safe_mode_ = vector<unsigned char> {0x00A};
    bool current_state_valid_ = false;
    bool desired_state_valid_ = false;
    bool use_position_ = true;
    bool service_done_ = false;
    bool stabilize_robot_ = true;

    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                // WAIT FOR VALID DATA TO INITIALIZE PIDs // 
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    void initDesiredState()
    {
        auto desiredValid = std::bind(&Controller::desiredStateValid, this);
        std::future<bool> promise = std::async(desiredValid);
        std::cout << "Waiting for Valid Sensor Info. \n";
        bool valid = promise.get();
        std::cout << "Got Valid Sensor Info. \n";
    }

    bool desiredStateValid()
    {
        while (!this->current_state_valid_)
        {
            this->sendNothingAndWait();
            std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME));
        }
        this->resetState();
        std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME));
        this->desired_state_ = vector<float>{0,0,0,0,0,0};
        while (!this->desired_state_valid_) {this->desired_state_valid_ = true;}
        while (!this->current_state_valid_) {this->current_state_valid_ = true;}
        this->pidReady();
        return true;
    }

    void printCurrentAndDesiredStates()
    {
        std::cout << "DESIRED STATE: ";
        printVector(this->desired_state_);
        std::cout << "CURRENT STATE: ";
        printVector(this->current_state_);
    }

    void sendNothingAndWait()
    {
        canClient::sendFrame(0x00A, 0, safe_mode_.data(), can_client_); // Keep from exiting safe mode by sending 0 command
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                        // UPDATES OF STATE FOR PIDs //
    /////////////////////////////////////////////////////////////////////////////////////////////////////


    /* Different timer for printing and sensor updates */
    void print_timer_callback()
    {
        cout << "\n_________________________\n";
        this->printCurrentAndDesiredStates();
        cout << "___________________________\n\n";
        this->controller_.getStatus(); 
        stabilize_robot_ ? cout << "Stabilizing Robot" << " | " : cout << "Not Stabilizing Robot" << " | ";
        current_state_valid_ && desired_state_valid_ ? cout << "ROBOT IS UPDATING" << " | " : cout << "WAITING FOR GOOD SENSOR INFO" << " | ";
        use_position_ ? cout << "USING POSITION" << " | " : cout << "IGNORING POSITION" << endl;
    }

    /* Essential callback set to update PID state at certain interval */
    void update_timer_callback()
    {
        update_current_state();
    }

    void update_current_state() // ** Important Function **
    {    
    /* 
     * STEP 1: Update the PID Controller (meaning call the ScionPIDController object's
     * update function which generates ctrl_vals and show its status on the screen 
     */
        sendNothingAndWait();
        if (stabilize_robot_ && current_state_valid_ && desired_state_valid_)
        {
            vector<float> thrusts(motor_count_, 0);
            thrusts = this->getThrusts(this->current_state_, this->desired_state_);
            make_CAN_request(thrusts);
        }
    }

    vector<float> getErrors(vector<float> current_state, vector<float> desired_state) 
    {
        return desired_state - current_state;
    }    

    vector<float> getThrusts(vector<float>& current_state, vector<float>& desired_state)
    {
        vector<float> errors                   {0.0F,0.0F,0.0F,0.0F,0.0F,0.0F}; 
        vector<float> adjustedErrors           {0.0F,0.0F,0.0F,0.0F,0.0F,0.0F};
        vector<float> ctrl_vals                {0.0F,0.0F,0.0F,0.0F,0.0F,0.0F}; 

        errors = getErrors(current_state,  desired_state);
        adjustedErrors = adjustErrors(errors);
        ctrl_vals = this->controller_.update(adjustedErrors, (float)UPDATE_PERIOD_RAW / 1000);
        return ctrlValsToThrusts(ctrl_vals);
    }    

    vector<float> ctrlValsToThrusts(vector<float>& ctrl_vals)
    {
        vector<float> thrusts = this->thrust_mapper_ * ctrl_vals;
        return thrusts;
    }

    float angleBetweenHereAndPoint(float x, float y)
    {
        if (areEqual(y, 0, .01)) {return 90;}
        float point_angle_radians = atan(x / y);
        float point_angle_degrees = point_angle_radians * (180/PI);
        if (y < 0) {point_angle_degrees += 180;}
        return point_angle_degrees;
    }

    float distanceBetweenHereAndPoint(float x, float y)
    {
        return sqrt(pow(x,2) + pow(y,2));
    }

    vector<float> adjustErrors(vector<float>& errors)
    {
        float yaw = current_state_[0];

        float x = errors[3];
        float y = errors[4];
        float z = errors[5];

        float absoluteAngle = angleBetweenHereAndPoint(y ,x);
        float absoluteDistance = distanceBetweenHereAndPoint(y, x);

        float yawAdjustedX = absoluteDistance * cos((absoluteAngle - yaw) * M_PI/180);
        float yawAdjustedY = absoluteDistance * sin((absoluteAngle - yaw) * M_PI/180);

        float adjustedX = yawAdjustedX;
        float adjustedY = yawAdjustedY;
        float adjustedZ = z;
                
        vector<float> adjustedErrors = vector<float>
        {
            errors[0],
            errors[1],
            errors[2],
            adjustedX,
            adjustedY,
            adjustedZ
        };
        return adjustedErrors;
    }

    vector<float> update_PID(Interface::current_state_t& current_state, Interface::desired_state_t& desired_state)
    {
        using namespace Interface;
        if (!this->current_state_valid_ || !this->desired_state_valid_) 
        {
            return vector<float>(this->motor_count_, 0);
        }

        if (this->use_position_)
        {
            return this->getThrusts(current_state, desired_state); 
        }
        else
        {
            current_state_t current_state_no_position = current_state_t{current_state[0], current_state[1], current_state[2], 0, 0, 0};
            desired_state_t desired_state_no_position = desired_state_t{desired_state[0], desired_state[1], desired_state[2], 0, 0, 0};
            return this->getThrusts(current_state_no_position, desired_state_no_position);    
        }
    
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                    // CAN REQUESTS FOR MOTOR CONTROL // 
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    vector<int> make_CAN_request(vector<float>& thrusts)
    {
        /* Thrusts come out of PID as a float between -1 and 1; motors need int value from -100 to 100 */
        std::vector<int> convertedThrusts;
        for (float thrust : thrusts)
        {
            convertedThrusts.push_back(((int)(thrust * 100)));
        }

        /* 
         * We have integer values that are 32 bits (4 bytes) but need values of one byte to send to motor
         * We can extract using an and mask and get last 8 bits which in hex is 0xFF. Char size is one byte
         * which is why we use an array of chars
         */          

        std::vector<unsigned char> byteThrusts;
        for (int thrust : convertedThrusts)
        {
            byteThrusts.push_back((thrust & 0xFF));
        }
        /* See exactly our 8 thrust values sent to motors */
        cout << "THRUSTS: ";
        printVector(convertedThrusts);

    ////////////////////////////////////////// BUILD REQUEST //////////////////////////////////////////
        /* 
         * Our frame will send CAN request 
         * one byte for each value -100 to 100 
         */

        canClient::sendFrame(MOTOR_ID, motor_count_, byteThrusts.data(), this->can_client_);
        return convertedThrusts;
    }

    ////////////////////////////////////////// END REQUEST //////////////////////////////////////////

    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                            // ACTION SERVER // 
    /////////////////////////////////////////////////////////////////////////////////////////////////////


    rclcpp_action::GoalResponse handle_goal
    (
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const PIDAction::Goal> goal
    )
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->desired_state);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel
    (
        const std::shared_ptr<GoalHandlePIDAction> goal_handle
    )
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted
    (
        const std::shared_ptr<GoalHandlePIDAction> goal_handle
    )
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&Controller::execute, this, _1), goal_handle}.detach();
    }

    bool areEqual(float float1, float float2, float epsilon)
    {
        return (fabs(float1 - float2) < epsilon);
    }

    bool areEqual(std::vector<float>& current_state, std::vector<float>& desired_state)
    {
        #define ORIENTATION_TOLERANCE 4.0f
        #define POSITION_TOLERANCE 0.05f

        bool equal = true;
        for (std::vector<float>::size_type i = 0; i < 3; i++)
        {
            if (!areEqual(current_state[i], desired_state[i], ORIENTATION_TOLERANCE)) //.05*current_state[i])
            {
                equal = false;
            }
        }
        for (std::vector<float>::size_type j = 3; j < 6; j++)
        {
            if (!areEqual(current_state[j], desired_state[j], POSITION_TOLERANCE)) //.05*current_state[i])
            {
                equal = false;
            }
        }
        return equal;
    }

    bool equalToZero(vector<int> thrustVect)
    {
        bool equal = true;
        for (int thrust : thrustVect)
        {
            if (thrust > 2)
            {
                equal = false;
            }
        }
        return equal;
    }


    void execute(const std::shared_ptr<GoalHandlePIDAction> goal_handle)
    {
        /* Goal Initialization */
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(20);

        std::shared_ptr<PIDAction::Feedback> feedback = std::make_shared<PIDAction::Feedback>();
        std::shared_ptr<PIDAction::Result> result = std::make_shared<PIDAction::Result>();
        const auto goal = goal_handle->get_goal();

        this->desired_state_ = goal->desired_state;

        /* Init States */
        std::vector<float>& state = feedback->current_state;
        vector<float> thrusts = this->update_PID(this->current_state_, this->desired_state_);
        vector<int> thrustInts = this->make_CAN_request(thrusts);
        for (int thrust : thrustInts)
        {
            state.push_back((float)thrust);
        }

        /* Feedback Loop - Stop Conditions is that all Thrusts are close to zero*/
        while (!this->equalToZero(thrustInts)) {
        //   Check if there is a cancel request
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }

        /* Update at Every Loop */
        thrusts = update_PID(this->current_state_, this->desired_state_);
        thrustInts = this->make_CAN_request(thrusts);
        for (int i = 0; i < thrustInts.size(); i++)
        { 
            state[i] = ((float)thrustInts[i]);
        }

        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
        }

        /* 
                    ARCHIVE
         // vector<float> target = vector<float>{current_state_[0], current_state_[1], current_state_[2],0,0,0};
        // int i = 0;
        // while (!areEqual(this->current_state_, target) && i < 3)
        // {   
        //     this->resetPosition();
        //     std::this_thread::sleep_for(std::chrono::milliseconds(300));
        //     i++;
        // }

        std::vector<float> desired_state = goal->desired_state;
        desired_state += current_state_;
        this->desired_state_ = desired_state; */
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                            // CONTROL SERVICES // 
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    void stopRobot(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        unsigned char can_dreq_frame[2] = {0, 0};             
        canClient::sendFrame(MOTOR_ID, motor_count_, can_dreq_frame, this->can_client_);
        this->desired_state_ = this->current_state_;
    }

    void pidReady()
    {
        auto pid_ready_request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto pid_ready_future = this->pid_ready_client_->async_send_request(pid_ready_request);
    }

    void resetState()
    {
        auto reset_state_request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto reset_state_future = this->reset_relative_state_client_->async_send_request(reset_state_request);
    }

    void resetPosition()
    {     
      auto reset_position_request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto reset_position_future = this->reset_relative_position_client_->async_send_request(reset_position_request);
    }

    void usePosition(const  std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                            std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        this->use_position_ = request->data;
    }

    void stabilizeRobot(const   std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        this->stabilize_robot_ = request->data;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                        // SUBSCRIPTION CALLBACKS // 
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    // void desired_state_callback(const scion_types::msg::State::SharedPtr msg)
    // /** 
    //  * You can use this subscription to manually send a desired state at command line
    //  **/ 
    // {
    //     if (!this->desired_state_valid_) {this->desired_state_valid_ = true;}
    //     // RCLCPP_INFO(this->get_logger(), "Received Desired Position Data:");
    //     printVector(msg->state);
    //     this->desired_state_= msg->state; 
    // }

    void current_state_callback(const scion_types::msg::State::SharedPtr msg)
    {
        if (!this->current_state_valid_) {this->current_state_valid_ = true;}
        this->current_state_= msg->state; 
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                        // GUI TUNING CONTROLS CALLBACKS // 
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    void kp_tuning_callback(const scion_types::msg::PidTuning::SharedPtr msg) 
    {
        shared_ptr<PID_Controller> axis = this->controller_.controllers[axis_tuning_map_[msg->axis]];
        float scale_factor = axis->angle_wrap ? .0001 : .001;
        axis->set_gains(scale_factor * msg->data, axis->k_i, axis->k_d);
    }

    void ki_tuning_callback(const scion_types::msg::PidTuning::SharedPtr msg) 
    {
        shared_ptr<PID_Controller> axis = this->controller_.controllers[axis_tuning_map_[msg->axis]];
        float scale_factor = axis->angle_wrap ? .0001 : .001;
        axis->set_gains(axis->k_p, scale_factor * msg->data, axis->k_d);
    }

    void kd_tuning_callback(const scion_types::msg::PidTuning::SharedPtr msg) 
    {
        shared_ptr<PID_Controller> axis = this->controller_.controllers[axis_tuning_map_[msg->axis]];
        float scale_factor = axis->angle_wrap ? .0001 : .001;
        axis->set_gains(axis->k_p, axis->k_i, scale_factor * msg->data);
    }
};

int main(int argc, char * argv[])
{   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>()); // simply call Controller constructor
    rclcpp::shutdown();
    return 0;
}
















/* ARCHIVE 
        // float pitch = current_state_[1];
        // float roll =  current_state_[2];
    // float pitchAdjustedX = absoluteDistance * cos(absoluteAngle - pitch);
        // float pitchAdjustedZ = absoluteDistance * sin(absoluteAngle - pitch);

        // float rollAdjustedY = absoluteDistance * cos(absoluteAngle - roll);
        // float rollAdjustedZ = absoluteDistance * sin(absoluteAngle - roll);
//, pitchAdjustedX
        // distanceBetweenHereAndPoint(yawAdjustedY, rollAdjustedY);
        // distanceBetweenHereAndPoint(rollAdjustedZ, pitchAdjustedZ); */