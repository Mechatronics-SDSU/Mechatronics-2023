/*
 * @author Conner Sommerfield - Zix on Discord
 * Brain node will send sequence of commands to the PIDs 
 * Commands can be pre-loaded or create with navigation logic
 * 
 * Robot will keep track of a "command queue"
 * This command queue will be abstracted as an vector/queue of Commands
 * Each command will be a function pointer to the action to perform 
 * and any parameters to pass to the function
 * 
 * The main ROS2 purpose of this node is to send a desired state to the
 * PIDs. It's the brain that tells the PIDs where the robot wants to go.
 * 
 * It will do this by taking the next out of the queue
 * 
 * A decision maker will be responsible for loading commands into the queue
 * 
 * PIDs will have to send command completion status for the queue mediator
 * to take out the next command.
 * 
 */

#include "rclcpp/rclcpp.hpp"
#include "scion_types/msg/desired_state.hpp"
#include "scion_types/msg/orientation.hpp"         // Custom message types defined in scion_types package
#include "scion_types/msg/position.hpp"            
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "vector_operations.hpp"                   
#include <stdlib.h>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;

#define STATE_UPDATE_PERIOD 120ms
#define MESSAGE_UPDATE_PERIOD 4s


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                    // NODE MEMBER VARIABLE DECLARIONS/INITIALIZATIONS //
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////


/* Brain will subscribe to position and orientation, and take that into account for publishing the desired state */

class Brain : public rclcpp::Node
{

struct Command
{
    vector<float>(Brain::*commandPtr)(float, vector<float>&);           // Points to a function to execute          (turn)
    float degree;                                                       // The magnitude to pass into that function (30 degrees)
};

public:
    Brain(): Node("brain_node")
    {
        state_timer_ = this->create_wall_timer(STATE_UPDATE_PERIOD, std::bind(&Brain::state_timer_callback, this));
        message_timer_ = this->create_wall_timer(MESSAGE_UPDATE_PERIOD, std::bind(&Brain::message_timer_callback, this));

        position_sub_ = this->create_subscription<scion_types::msg::Position>
        ("zed_position_data", 10, std::bind(&Brain::position_sub_callback, this, _1));

        orientation_sub_ = this->create_subscription<scion_types::msg::Orientation>
        ("ahrs_orientation_data", 10, std::bind(&Brain::orientation_sub_callback, this, _1));
        
        desired_state_pub_ = this->create_publisher<scion_types::msg::DesiredState>
        ("desired_state_data", 10);

        // reset_pos_client_ = this->create_client<std_srvs::srv::Trigger>("/zed2i/zed_node/reset_pos_tracking");

        forwardBack(command_sequence_);                                   // Set command sequence with the commands defined in custom command sequence function
    }

private:
    rclcpp::Subscription<scion_types::msg::Position>::SharedPtr position_sub_;
    rclcpp::Subscription<scion_types::msg::DesiredState>::SharedPtr desired_state_sub_;
    rclcpp::Publisher<scion_types::msg::DesiredState>::SharedPtr desired_state_pub_;
    rclcpp::Subscription<scion_types::msg::Orientation>::SharedPtr orientation_sub_;
    // rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_pos_client_; 
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr message_timer_;
    vector<Command> command_sequence_;
    int counter_ = 0;


    /* Upon initialization set all values to [0,0,0...] */
    vector<float> current_orientation_{0.0F,0.0F,0.0F};
    vector<float> current_position_{0.0F,0.0F,0.0F};
    vector<float> current_state_{0.0F,0.0F,0.0F,0.0F,0.0F,0.0F};        // State described by yaw, pitch, roll, x, y, z 

    vector<float> desired_state_ = current_state_;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                        // COMMAND SEQUENCE DEFINTIONS //
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // void setCommandSequence(&command_sequence_)
    // {
    //     command_sequence_ = turnInBox();
    // }

    void turnInBox(vector<Command>& command_sequence)
    {
        /* 
         * Turn in Box command by a loop of going forward and then turning 90 degrees continually
         */

        Command command1;
        command1.commandPtr = &Brain::move;
        command1.degree = 0.3;

        Command command2;
        command2.commandPtr = &Brain::turn; //static_cast<Command*>
        command2.degree = 90.0;

        command_sequence.push_back(command1);
        command_sequence.push_back(command2);
        
        /* View our commands */
        for (Command command : command_sequence)
        {
            std::cout << (void*)command.commandPtr << std::endl;
            std::cout << command.degree << std::endl;
        }
    }

    void forwardBack(vector<Command>& command_sequence)
    {
        Command command1;
        command1.commandPtr = &Brain::move;
        command1.degree = 0.5;

        Command command2;
        command2.commandPtr = &Brain::move; //static_cast<Command*>
        command2.degree = -0.5;

        command_sequence.push_back(command1);
        command_sequence.push_back(command2);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                    // SERVICE REQUEST TO RESET POSITION //
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // template<typename T>
    // auto makeRequest() 
    // /** 
    //  * Reset the position of Zed to 0,0,0
    // **/
    // {
    //     auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

        
    //     while (!reset_pos_client_->wait_for_service(1s)) 
    //     {
    //         if (!rclcpp::ok()) 
    //         {
    //             RCLCPP_ERROR(rclcpp::get_logger("brain_node"), "Interrupted while waiting for the service. Exiting.");
    //         }
    //         RCLCPP_INFO(rclcpp::get_logger("brain_node"), "service not available, waiting again...");
    //     }
    //     auto result = reset_pos_client_->async_send_request(request);
    //     return result;
    // }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                    // GENERATE DESIRED STATE AND SEND TO PID  //
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void message_timer_callback()
    {
        /* 
         * Executes on a timer, sends the desired_state to the PID 
         * TODO: Keep on a short timer but with condition of 1st command completion to move to the 2nd command
         */

        auto message = scion_types::msg::DesiredState();
        message.desired_state = constructDesiredState();    // Other function will be called to provide logic of desired state (decides where to go)
        desired_state_ = message.desired_state;             // Just to print to the console the desired state
        desired_state_pub_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing Desired State " );
    }
    
    std::vector<float> constructDesiredState()
    {
        /* 
         * Grabs a command from the command vector and extracts the appropriate 
         * desired state based on the command to send to the publisher. 
         * Desired state is a vector of 6 floats. 
         */
        std::vector<float> desired_state; 
        Command command = command_sequence_[(counter_ % command_sequence_.size())];

        ////////////// TESTING OUTPUT TO CONSOLE ////////////
        cout << "ptr:" << (void*)command.commandPtr << " " << std::endl;
        cout << "float: "<< command.degree << " " << std::endl;
        cout << "vector: " << std::endl;
        printVector(current_state_);
        ////////////////////////////////////////////////////


        vector<float> (Brain::*command_function)(float, vector<float>&) = command.commandPtr;           // Extract function pointer from Command struct
        desired_state = (this->*command_function)(command.degree, current_state_);                      // Actual function call using degree parameter from Command and current_state_ member

        counter_ = counter_ + 1;
        return desired_state;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                // TRACKING SENSOR INFO AND CURRENT STATE  //
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void state_timer_callback()
    /* Essential callback set to update current sensor state at certain interval */
    {
        update_current_state();
        printVector(current_state_);
        printVector(desired_state_);
    }

    void update_current_state()
    {
    /*
     * STEP 1: Build current state from current information 
     * that has been updated from the sensors. As of now
     * we are using orientation as first 3 values, position as next 3
     */
        // this->current_state_[0] = this->current_orientation_[0];
        // this->current_state_[1] = this->current_orientation_[1];
        // this->current_state_[2] = this->current_orientation_[2];
        this->current_state_[3] = this->current_position_[0];
        this->current_state_[4] = this->current_position_[1];
        this->current_state_[5] = this->current_position_[2];
    }

    void orientation_sub_callback(const scion_types::msg::Orientation::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received ahrs_orientation_data");
        this->current_orientation_ = msg->orientation;
    }

    void position_sub_callback(const scion_types::msg::Position::SharedPtr msg)
        {    
            RCLCPP_INFO(this->get_logger(), "Received Zed Position Data");
            this->current_position_  = msg->position;
        }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                    // POSSIBLE FUNCTIONS TO BE PERFORMED  //
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    vector<float> turn(float degrees, vector<float>& current_state_)
    {
        return vector<float>{degrees + current_state_[0], current_state_[1],current_state_[2], current_state_[3], current_state_[4], current_state_[5]};
    }

    vector<float> move(float distance, vector<float>& current_state_)
    {
        // makeRequest();
        return vector<float>{current_state_[0], current_state_[1], current_state_[2], distance + current_state_[3], current_state_[4], current_state_[5]};
    }
};

int main(int argc, char * argv[])
{   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Brain>()); // simply call Brain constructor
    rclcpp::shutdown();
    return 0;
}
 
