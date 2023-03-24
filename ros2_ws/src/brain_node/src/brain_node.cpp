/*
 * @author Conner Sommerfield - Zix on Discord
 * Brain node will send sequence of commands to the PIDs 
 * Commands can be pre-loaded or create with navigation logic
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
#define MESSAGE_UPDATE_PERIOD 3s


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                    // NODE MEMBER VARIABLE DECLARIONS/INITIALIZATIONS //
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////

class Brain : public rclcpp::Node
{


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

        turnInBox(command_sequence_); 
    }

struct Command
{
    vector<float>(Brain::*fun_ptr)(float, vector<float>&);
    float param1;
    // vector<float>* param2;
};

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
    vector<float> current_state_{0.0F,0.0F,0.0F,0.0F,0.0F,0.0F}; // State described by yaw, pitch, roll, x, y, z 

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
        Command command1;
        command1.fun_ptr = &Brain::move;
        command1.param1 = 0.3;
        // command1.param2 = &current_state_;

        Command command2;
        command2.fun_ptr = &Brain::turn; //static_cast<Command*>
        command2.param1 = 90.0;
        // command2.param2 = &current_state_;

        command_sequence.push_back(command1);
        command_sequence.push_back(command2);
        
        for (Command command : command_sequence)
        {
            std::cout << (void*)command.fun_ptr << std::endl;
            std::cout << command.param1 << std::endl;
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                    // SERVICE REQUEST TO RESET POSITION //
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // template<typename T>
    // auto makeRequest() 
    // /** 
    //  * request containing unixtime will be built from message as they are identical, request sent to Node2
    //  * client will wait for the service (Node 2) to convert unixtime to a date and capture response
    //  * Returns this response to print later.
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
        auto message = scion_types::msg::DesiredState();
        message.desired_state = constructDesiredState();
        desired_state_ = message.desired_state; // Just to print to the console the desired state
        desired_state_pub_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing Desired State " );
    }
    
    std::vector<float> constructDesiredState()
    {
        // for (Command* commandPtr : command_sequence_)
        // {
        //     std::cout << commandPtr->param1;
        //     printVector(commandPtr->param2);
        // }
        std::vector<float> desired_state; 
        Command command = command_sequence_[(counter_ % command_sequence_.size())];
        cout << "ptr:" << (void*)command.fun_ptr << " " << std::endl;
        cout << "float: "<< command.param1 << " " << std::endl;
        cout << "vector: " << std::endl;
        printVector(current_state_);

        vector<float> (Brain::*command_function)(float, vector<float>&) = command.fun_ptr;
        desired_state = (this->*command_function)(command.param1, current_state_);         
        counter_ = counter_ + 1;
        return desired_state;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                    // TRACKING SENSOR INFO AND CURRENT STATE  //
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void state_timer_callback()
    /* Essential callback set to update PID state at certain interval */
    {
        // makeRequest()
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
        this->current_state_[0] = this->current_orientation_[0];
        this->current_state_[1] = this->current_orientation_[1];
        this->current_state_[2] = this->current_orientation_[2];
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
 
