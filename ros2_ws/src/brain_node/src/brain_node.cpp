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

    /* Upon initialization set all values to [0,0,0...] */
    vector<float> current_orientation_{0.0F,0.0F,0.0F};
    vector<float> current_position_{0.0F,0.0F,0.0F};
    vector<float> current_state_{0.0F,0.0F,0.0F,0.0F,0.0F,0.0F}; // State described by yaw, pitch, roll, x, y, z 

    void state_timer_callback()
    /* Essential callback set to update PID state at certain interval */
    {
        update_current_state();
        printVector(current_state_);
    }

    void message_timer_callback()
    {
        auto message = scion_types::msg::DesiredState();
        message.desired_state = std::vector<float>{0,0,0,0,0,0};
        message.desired_state = constructDesiredState();
        desired_state_pub->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing Desired State " );
    }

    std::vector<float> constructDesiredState()
    {
        
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

};

int main(int argc, char * argv[])
{   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Brain>()); // simply call Brain constructor
    rclcpp::shutdown();
    return 0;
}
 
