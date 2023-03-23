#include "rclcpp/rclcpp.hpp"
#include "scion_types/msg/desired_state.hpp"
#include "scion_types/msg/orientation.hpp"         // Custom message types defined in scion_types package
#include "scion_types/msg/position.hpp"            
#include "std_msgs/msg/float32.hpp"
#include "vector_operations.hpp"                   



using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;

#define STATE_UPDATE_PERIOD 120ms
#define MESSAGE_UPDATE_PERIOD 3s


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
        
        desired_state_pub = this->create_publisher<scion_types::msg::DesiredState>
        ("desired_state_data", 10);
    }

private:
    rclcpp::Subscription<scion_types::msg::Position>::SharedPtr position_sub_;
    rclcpp::Subscription<scion_types::msg::DesiredState>::SharedPtr desired_state_sub_;
    rclcpp::Publisher<scion_types::msg::DesiredState>::SharedPtr desired_state_pub;
    rclcpp::Subscription<scion_types::msg::Orientation>::SharedPtr orientation_sub_;
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr message_timer_;
    int counter_ = 0;
    vector<int> commandSequence_{0, 1, 0, 1};

    /* Upon initialization set all values to [0,0,0...] */
    vector<float> current_orientation_{0.0F,0.0F,0.0F};
    vector<float> current_position_{0.0F,0.0F,0.0F};
    vector<float> current_state_{0.0F,0.0F,0.0F,0.0F,0.0F,0.0F}; // State described by yaw, pitch, roll, x, y, z 

    vector<float> desired_state_ = current_state_;

    void state_timer_callback()
    /* Essential callback set to update PID state at certain interval */
    {
        update_current_state();
        printVector(current_state_);
        printVector(desired_state_);
    }

    void message_timer_callback()
    {
        auto message = scion_types::msg::DesiredState();
        message.desired_state = std::vector<float>{0,0,0,0,0,0};
        message.desired_state = constructDesiredState(commandSequence_[counter_]);
        desired_state_ = message.desired_state;
        // message.desired_state = desired_state_;
        desired_state_pub->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing Desired State " );
        counter_ = ((counter_ + 1) % 4);
    }

    std::vector<float> constructDesiredState(int command)
    {
        if (command == 0)
        {
            return move(.3, current_state_);
        }

        else
        {
            return turn(90, current_state_);
        }
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

    vector<float> turn(float degrees, vector<float> current_state_)
    {
        return vector<float>{degrees, current_state_[1],current_state_[2], current_state_[3], current_state_[4], current_state_[5]};
    }

    vector<float> move(float distance, vector<float> current_state_)
    {
        return vector<float>{current_state_[0], current_state_[1], current_state_[2], distance, current_state_[4], current_state_[5]};
    }

};


// std::vector<void (*)()> turnSequence;           // vector of functions holds the desired turn sequence

// SomeClass::addThingy(void (*function)())
// {
//     //Don't take the address of the address:
//     vectoroffunctions.push_back(function);
// }



int main(int argc, char * argv[])
{   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Brain>()); // simply call Brain constructor
    rclcpp::shutdown();
    return 0;
}
 
