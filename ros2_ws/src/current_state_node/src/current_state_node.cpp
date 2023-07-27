#include "current_state_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;

/* Subscribe to all relevant sensor information and consolidate it for the PID Node to subscribe to */

CurrentStateNode::CurrentStateNode() : Node("current_state_node")
{
    this->declare_parameter("use_position_tracking", true);
    this->declare_parameter("use_orientation_tracking", true);

    bool position = this->get_parameter("use_position_tracking").get_parameter_value().get<bool>();
    bool orientation =  this->get_parameter("use_orientation_tracking").get_parameter_value().get<bool>();
    if (position)
    {
        position_valid_ = false;
    }
    if (orientation)
    {
        orientation_valid_ = false;
    }

   
    orientation_sub_ = this->create_subscription<scion_types::msg::State>
    ("ahrs_orientation_data", 10, std::bind(&CurrentStateNode::orientation_sub_callback, this, _1));


    a50_state_sub_ = this->create_subscription<scion_types::msg::State>
    ("a50_state_data", 10, std::bind(&CurrentStateNode::a50_state_sub_callback, this, _1));

    
    state_pub_timer_ = this->create_wall_timer
            (
                std::chrono::milliseconds(50), 
                std::bind(&CurrentStateNode::publish_state, this)
            );

    absolute_state_pub_ =           this->create_publisher<scion_types::msg::State>("absolute_current_state_data", 10);
    relative_state_pub_ =           this->create_publisher<scion_types::msg::State>("relative_current_state_data", 10);
    reset_relative_state_service_ = this->create_service<std_srvs::srv::Trigger>("reset_relative_state", std::bind(&CurrentStateNode::resetRelativeState, this, _1, _2));
    reset_relative_position_service_ = this->create_service<std_srvs::srv::Trigger>("reset_relative_position", std::bind(&CurrentStateNode::resetRelativePosition, this, _1, _2));

    auto initFunction = std::bind(&CurrentStateNode::initCurrentState, this);
    std::thread(initFunction).detach();
}

void CurrentStateNode::initCurrentState()
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

bool CurrentStateNode::currentStateValid()
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


void CurrentStateNode::update_current_state()
{
    this->current_state_[0] = this->current_orientation_[0];
    this->current_state_[1] = this->current_orientation_[1];
    this->current_state_[2] = this->current_orientation_[2];
    this->current_state_[3] = this->current_position_[0];
    this->current_state_[4] = this->current_position_[1];
    this->current_state_[5] = this->current_position_[2];
}

void CurrentStateNode::publish_absolute_state()
{
    scion_types::msg::State absolute_state = scion_types::msg::State();
    absolute_state.state = this->current_state_;
    this->absolute_state_pub_->publish(absolute_state);
}

void CurrentStateNode::publish_relative_state()
{
    scion_types::msg::State relative_state = scion_types::msg::State();
    relative_state.state = this->current_state_ - this->relative_state_;
    this->relative_state_pub_->publish(relative_state);
}

void CurrentStateNode::publish_state()
{
    this->update_current_state();
    if (current_state_valid_)
    {
        this->publish_absolute_state();
        this->publish_relative_state();
    }
}

void CurrentStateNode::resetRelativeState     (
                            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response
                            )
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming Request to Reset Relative State\n");
    this->relative_state_ = this->current_state_;
    response->success = true;
}

void CurrentStateNode::resetRelativePosition  (
                            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response
                            )
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming Request to Reset Relative Position\n");
    this->relative_state_[3] = this->current_state_[3];
    this->relative_state_[4] = this->current_state_[4];
    this->relative_state_[5] = this->current_state_[5];
    response->success = true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
                                    // SUBSCRIPTION CALLBACKS // 
/////////////////////////////////////////////////////////////////////////////////////////////////////

/* 
    * These callbacks all update a member variable that holds current state of PID, each corresponds to a 
    * sensor. When that sensor publishes, the PID will store the last sensed value  
    */

void CurrentStateNode::orientation_sub_callback(const scion_types::msg::State::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Received ahrs_orientation_data");
    if (!this->orientation_valid_) {orientation_valid_ = true;}
    this->current_orientation_ = msg->state;
}

void CurrentStateNode::a50_state_sub_callback(const scion_types::msg::State::SharedPtr msg)
{    
        RCLCPP_INFO(this->get_logger(), "Received a50 Position Data");
        if (!this->position_valid_) {position_valid_ = true;}
        this->current_position_  = msg->state;
}



 // position_sub_ = this->create_subscription<scion_types::msg::Position>
    // ("zed_position_data", 10, std::bind(&CurrentStateNode::position_sub_callback, this, _1));

    // depth_sub_ = this->create_subscription<scion_types::msg::Datapoint>
    // ("ms5837_depth", 10, std::bind(&CurrentStateNode::depth_sub_callback, this, _1));

    // velocity_sub_ = this->create_subscription<scion_types::msg::Orientation>
    // ("dvl_velocity_data", 10, std::bind(&Controller::velocity_sub_callback, this, _1));
// dvl_position_sub_ = this->create_subscription<scion_types::msg::State>
    // ("dvl_position_data", 10, std::bind(&CurrentStateNode::dvl_position_sub_callback, this, _1));

// void CurrentStateNode::position_sub_callback(const scion_types::msg::Position::SharedPtr msg)
// {    
//     //  RCLCPP_INFO(this->get_logger(), "Received Zed Position Data");
//      if (!this->position_valid_) {position_valid_ = true;}
//      this->current_position_  = msg->position;
// }

// void CurrentStateNode::depth_sub_callback(const scion_types::msg::Datapoint::SharedPtr msg)
// {
//     if (!this->position_valid_) {position_valid_ = true;}
//     RCLCPP_INFO(this->get_logger(), "Received ms5837 depth data: %f", msg->data);
//     this->current_position_ = vector<float>{0.0, 0.0, msg->data};
// }

// void CurrentStateNode::velocity_sub_callback(const std_msgs::msg::Float32::SharedPtr msg)
// {
//     RCLCPP_INFO(this->get_logger(), "Received ms5837 depth data: %f", msg->data);
//     this->current_position_ = vector<float>{0.0, 0.0, msg->data} ;
// }

// void CurrentStateNode::dvl_position_sub_callback(const scion_types::msg::State::SharedPtr msg)
// {    
//     //  RCLCPP_INFO(this->get_logger(), "Received Zed Position Data");
//      if (!this->position_valid_) {position_valid_ = true;}
//      this->current_position_  = msg->state;