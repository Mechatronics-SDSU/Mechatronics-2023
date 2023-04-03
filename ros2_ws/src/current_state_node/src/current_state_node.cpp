#include <memory>
#include <vector>
#include <unistd.h>
#include <thread>
#include <future>
#include <iostream>
#include <functional>
#include <math.h>

#include "vector_operations.hpp"
#include "rclcpp/rclcpp.hpp"
#include "control_interface.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "scion_types/msg/state.hpp"
#include "scion_types/msg/position.hpp"
#include "scion_types/msg/orientation.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

#define ORIENTATION 
#define POSITION


/* Subscribe to all relevant sensor information and consolidate it for the PID Node to subscribe to */

class CurrentStateNode : public rclcpp::Node
{
  public:
    CurrentStateNode()
    : Node("current_state_node")
    {
        #ifdef ORIENTATION
            orientation_valid_ = false;
        #endif

        #ifdef POSITION
            position_valid_ = false;
        #endif

        position_sub_ = this->create_subscription<scion_types::msg::Position>
        ("zed_position_data", 10, std::bind(&CurrentStateNode::position_sub_callback, this, _1));

        orientation_sub_ = this->create_subscription<scion_types::msg::Orientation>
        ("ahrs_orientation_data", 10, std::bind(&CurrentStateNode::orientation_sub_callback, this, _1));
    
        // depth_sub_ = this->create_subscription<std_msgs::msg::Float32>
        // ("ms5837_depth_data", 10, std::bind(&Controller::depth_sub_callback, this, _1));

        // velocity_sub_ = this->create_subscription<scion_types::msg::Orientation>
        // ("dvl_velocity_data", 10, std::bind(&Controller::velocity_sub_callback, this, _1));

        state_pub_timer_ = this->create_wall_timer
                (
                    std::chrono::milliseconds(50), 
                    std::bind(&CurrentStateNode::publish_state, this)
                );

        absolute_state_pub_ =           this->create_publisher<scion_types::msg::State>("absolute_current_state_data", 10);
        relative_state_pub_ =           this->create_publisher<scion_types::msg::State>("relative_current_state_data", 10);
        reset_relative_state_service_ = this->create_service<std_srvs::srv::Trigger>("reset_relative_state", std::bind(&CurrentStateNode::resetRelativeState, this, _1, _2));

        auto initFunction = std::bind(&CurrentStateNode::initCurrentState, this);
        std::thread(initFunction).detach();
    }

  private:    
    rclcpp::Subscription<scion_types::msg::Position>::SharedPtr position_sub_;
    rclcpp::Subscription<scion_types::msg::Orientation>::SharedPtr orientation_sub_;
    rclcpp::Publisher<scion_types::msg::State>::SharedPtr absolute_state_pub_;
    rclcpp::Publisher<scion_types::msg::State>::SharedPtr relative_state_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_relative_state_service_;
    rclcpp::TimerBase::SharedPtr state_pub_timer_;
    Interface::current_state_t current_state_{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
    Interface::current_state_t relative_state_{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
    std::vector<float> current_orientation_{0.0F, 0.0F, 0.0F};
    std::vector<float> current_position_{0.0F, 0.0F, 0.0F};
    bool current_state_valid_ = false;
    bool orientation_valid_ = true;
    bool position_valid_ = true;
    
    void initCurrentState()
    {
        auto currValid = std::bind(&CurrentStateNode::currentStateValid, this);
        std::future<bool> promise = std::async(currValid);
        std::cout << "Waiting for Valid Sensor Info. \n";
        bool valid = promise.get();
        if (valid) 
        {
            std::cout << "Got Valid Sensor Info. \n";
        }
    }

    bool currentStateValid()
    {
        while (!this->current_state_valid_)
        {
            if (this->orientation_valid_ && this->position_valid_) 
            {
                this->current_state_valid_ = true;
            }
            sleep(.1);
        }
        return true;
    }

    void publish_state()
    {
        current_state_[0] = current_orientation_[0];
        current_state_[1] = current_orientation_[1];
        current_state_[2] = current_orientation_[2];
        current_state_[3] = current_position_[0];
        current_state_[4] = current_position_[1];
        current_state_[5] = current_position_[2];

        if (current_state_valid_)
        {
            scion_types::msg::State absolute_state = scion_types::msg::State();
            absolute_state.state = this->current_state_;
            absolute_state_pub_->publish(absolute_state);

            scion_types::msg::State relative_state = scion_types::msg::State();
            relative_state.state = this->current_state_ - this->relative_state_;
            relative_state.state[3] = sqrt(pow(relative_state.state[3], 2) + pow(relative_state.state[4], 2));
            relative_state.state[4] = 0;
            relative_state_pub_->publish(relative_state);
        }
    }

    void resetRelativeState     (
                                const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                      std::shared_ptr<std_srvs::srv::Trigger::Response> response
                                )
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming Request to Reset Relative Position\n");
        this->relative_state_ = this->current_state_;
    }
    
    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                        // SUBSCRIPTION CALLBACKS // 
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    /* 
     * These callbacks all update a member variable that holds current state of PID, each corresponds to a 
     * sensor. When that sensor publishes, the PID will store the last sensed value  
     */

    void orientation_sub_callback(const scion_types::msg::Orientation::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received ahrs_orientation_data");
        if (!this->orientation_valid_) {orientation_valid_ = true;}
        this->current_orientation_ = msg->orientation;
    }

    void position_sub_callback(const scion_types::msg::Position::SharedPtr msg)
    {    
         RCLCPP_INFO(this->get_logger(), "Received Zed Position Data");
         if (!this->position_valid_) {position_valid_ = true;}
         this->current_position_  = msg->position;
    }

    // void depth_sub_callback(const std_msgs::msg::Float32::SharedPtr msg)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Received ms5837 depth data: %f", msg->data);
    //     this->current_position_ = vector<float>{0.0, 0.0, msg->data} ;
    // }

    // void velocity_sub_callback(const std_msgs::msg::Float32::SharedPtr msg)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Received ms5837 depth data: %f", msg->data);
    //     this->current_position_ = vector<float>{0.0, 0.0, msg->data} ;
    // }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CurrentStateNode>());
  rclcpp::shutdown();
  return 0;
}