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
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <vector>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_interface.hpp"
#include "scion_types/action/pid.hpp"
#include "scion_types/msg/state.hpp"
#include "scion_pid_controller.hpp"                // PID Class
#include "mbox_can.hpp"                            // For CAN Requests

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;

#define MBOX_INTERFACE "can0"
#define UPDATE_PERIOD 50ms
#define PRINT_PERIOD 500ms
#define PID_ERROR_THRESHOLD 0.01f

/////////////////////////////////////////////////////////////////////////////////////////////////////
                                    // MEMBER VARIABLE DECLARATIONS // 
/////////////////////////////////////////////////////////////////////////////////////////////////////

using PIDAction = scion_types::action::PID;
using GoalHandlePIDAction = rclcpp_action::ServerGoalHandle<PIDAction>;
using namespace std::placeholders;


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
        timer_ = this->create_wall_timer(UPDATE_PERIOD, std::bind(&Controller::timer_callback, this));
        
        print_timer_ = this->create_wall_timer(PRINT_PERIOD, std::bind(&Controller::print_timer_callback, this));

        current_state_sub_ = this->create_subscription<scion_types::msg::State>
        ("relative_current_state_data", 10, std::bind(&Controller::current_state_callback, this, _1));

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

        controller_ = Scion_Position_PID_Controller(pid_params_object_.get_pid_params());
        controller_.getStatus();
        
        strncpy(ifr.ifr_name, MBOX_INTERFACE, sizeof(&MBOX_INTERFACE));
        poll_mb_ = new Mailbox::MboxCan(&ifr, "poll_mb");

        auto initFunction = std::bind(&Controller::initDesiredState, this);
        std::thread(initFunction).detach();
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr print_timer_;
    rclcpp::Subscription<scion_types::msg::State>::SharedPtr current_state_sub_;
    rclcpp::Subscription<scion_types::msg::State>::SharedPtr desired_state_sub_;
    rclcpp_action::Server<PIDAction>::SharedPtr pid_command_server_;
    Scion_Position_PID_Controller controller_;
    PID_Params pid_params_object_;                      // Passed to controller for tuning
    Mailbox::MboxCan* poll_mb_;
    struct ifreq ifr;

    /* Upon initialization set all values to [0,0,0] */
    
    Interface::current_state_t current_state_{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F}; // State described by yaw, pitch, roll, x, y, z 
    Interface::desired_state_t desired_state_{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F}; // Desired state is that everything is set to 0 except that its 1 meter below the water {0,0,0,0,0,1}
    bool current_state_valid_ = false;
    bool desired_state_valid_ = false;

    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                // WAIT FOR VALID DATA TO INITIALIZE PIDs // 
    /////////////////////////////////////////////////////////////////////////////////////////////////////

     void initDesiredState()
    {
        auto desiredValid = std::bind(&Controller::desiredStateValid, this);
        std::future<bool> promise = std::async(desiredValid);
        std::cout << "Waiting for Valid Sensor Info. \n";
        bool valid = promise.get();
        if (valid) 
        {
            std::cout << "Got Valid Sensor Info. \n";
        }
    }

    bool desiredStateValid()
    {
        while (!this->current_state_valid_)
        {
            sleep(.1);
        }
        // while (!this->areEqual(this->desired_state_, this->current_state_))
        for (int i = 0; i < 1000; i++)
        {
            this->desired_state_ = this->current_state_;
        }
        while (!this->desired_state_valid_)
        {
            this->desired_state_valid_ = true;
        }
        while (!this->current_state_valid_)
        {
            this->current_state_valid_ = true;
        }
        return true;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                        // UPDATES OF STATE FOR PIDs //
    /////////////////////////////////////////////////////////////////////////////////////////////////////


    /* Different timer for printing and sensor updates */
    void print_timer_callback()
    {
        std::cout << "PID STATE UPDATE\n _______________________________\n"; 
        std::cout << "Current State: ";
        printVector(this->current_state_);
        std::cout << "Desired State: ";
        printVector(this->desired_state_);
        std::cout << "___________________________\n\n";
        this->controller_.getStatus(); 
    }

    void timer_callback()
    /* Essential callback set to update PID state at certain interval */
    {
        update_current_state();
    }

    void update_current_state() // ** Important Function **
    {    
    /* 
     * STEP 1: Update the PID Controller (meaning call the ScionPIDController object's
     * update function which generates ctrl_vals and show its status on the screen 
     */

    std::cout << this->current_state_valid_ << std::endl;
    std::cout << this->desired_state_valid_ << std::endl;

                                        // Refer to classes/pid_controller/scion_pid_controller.hpp for this function
    if (current_state_valid_ && desired_state_valid_)
    {
        this->controller_.update(current_state_, desired_state_, .010); // MOST IMPORTANT LINE IN PROGRAM
    /* STEP 2: Send those generated values to the motors */
        make_CAN_request(this->controller_.current_thrust_values);
    }

    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                    // CAN REQUESTS FOR MOTOR CONTROL // 
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    void make_CAN_request(vector<float>& thrusts)
    {
        /* Thrusts come out of PID as a float between -1 and 1; motors need int value from -100 to 100 */
        int thrust0 = (int)(thrusts[0]*100);
        int thrust1 = (int)(thrusts[1]*100);
    
        /* See exactly our 8 thrust values sent to motors */
        std::cout << " " << thrust0 << " " << thrust1 << " ";
        std::cout << std::endl;

        /* 
         * We have integer values that are 32 bits (4 bytes) but need values of one byte to send to motor
         * We can extract using an and mask and get last 8 bits which in hex is 0xFF. Char size is one byte
         * which is why we use an array of chars
         */
        unsigned char can_dreq_frame[2] = 
                                {
                                    (thrust0 & 0xFF),
                                    (thrust1 & 0xFF),
                                };             

        // This is a manual motor test can frame that sets each motor to .1 potential
        // char can_dreq_frame[8] = {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10};

    ////////////////////////////////////////// BUILD REQUEST //////////////////////////////////////////
        /* 
         * Our frame will send CAN request 
         * one byte for each value -100 to 100 
         */

        struct can_frame poll_frame;
        poll_frame.can_id = 0x010;
        poll_frame.can_dlc = 2;
        std::copy(std::begin(can_dreq_frame),
                  std::end(can_dreq_frame),
                  std::begin(poll_frame.data));
        if(Mailbox::MboxCan::write_mbox(this->poll_mb_, &poll_frame) == -1) 
        {
            RCLCPP_INFO(this->get_logger(),
            "[DresDecodeNode::_data_request] Failed to write CAN Request.");
	    }
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
        #define ORIENTATION_TOLERANCE 1.0f
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
            if (!areEqual(current_state[i], desired_state[i], POSITION_TOLERANCE)) //.05*current_state[i])
            {
                equal = false;
            }
        }
        return equal;
    }

    void execute(const std::shared_ptr<GoalHandlePIDAction> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(100);

        std::shared_ptr<PIDAction::Feedback> feedback = std::make_shared<PIDAction::Feedback>();
        std::shared_ptr<PIDAction::Result> result = std::make_shared<PIDAction::Result>();
        const auto goal = goal_handle->get_goal();

        std::vector<float>& state = feedback->current_state;
        state.push_back(this->current_state_[0]);
        state.push_back(this->current_state_[1]);
        state.push_back(this->current_state_[2]);
        state.push_back(this->current_state_[3]);
        state.push_back(this->current_state_[4]);
        state.push_back(this->current_state_[5]);
        
        std::vector<float> desired_state = goal->desired_state;
        for (int i = 0; i < 100; i++)
        {
            desired_state += this->current_state_;
            this->desired_state_ = desired_state;
        }
        std::cout << "Goal State";
        printVector(desired_state);

        while (!areEqual(state, desired_state)) {
        //   Check if there is a cancel request
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }
        std::stringstream ss;
        // Update sequence
        state[0] = this->current_state_[0];
        state[1] = this->current_state_[1];
        state[2] = this->current_state_[2];
        state[3] = this->current_state_[3];
        state[4] = this->current_state_[4];
        state[5] = this->current_state_[5];

        // std::cout << "State according to Server";
        // printVector(state);

        // Publish feedback
        goal_handle->publish_feedback(feedback);
        // RCLCPP_INFO(this->get_logger(), "Publish feedback");

        loop_rate.sleep();

        }
        // Check if goal is done
        if (rclcpp::ok()) {
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
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
    /** 
     * You can use this subscription to manually send a desired state at command line
     **/ 
    {
        if (!this->current_state_valid_) {this->current_state_valid_ = true;}
        // RCLCPP_INFO(this->get_logger(), "Received Current Position Data:");
        printVector(msg->state);
        this->current_state_= msg->state; 
    }
};

int main(int argc, char * argv[])
{   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>()); // simply call Controller constructor
    rclcpp::shutdown();
    return 0;
}