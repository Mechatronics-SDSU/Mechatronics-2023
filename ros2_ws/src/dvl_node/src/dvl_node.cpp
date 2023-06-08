#include <vector>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "control_interface.hpp"
#include "scion_types/msg/datapoint.hpp"

using namespace std;

#define UPDATE_RATE 50

class DVL : public rclcpp::Node
{
    public:
        DVL()
        : Node("dvl_node")
        {
            vel_x_sub_ = this->create_subscription<scion_types::msg::DataPoint>
            ("dvl_vel_x", 10, std::bind(&Controller::vel_x_sub_callback, this, _1));

            vel_y_sub_ = this->create_subscription<scion_types::msg::DataPoint>
            ("dvl_vel_y", 10, std::bind(&Controller::vel_y_sub_callback, this, _1));

            position_pub_timer_ = this->create_wall_timer
                (
                    std::chrono::milliseconds(50), 
                    std::bind(&DVL::publish_position, this)
                );
            
            position_pub_ = this->create_publisher<scion_types::msg::Position>("dvl_position_data", 10);

            orientation_sub_ = this->create_subscription<scion_types::msg::Orientation>
            ("ahrs_orientation_data", 10, std::bind(&CurrentStateNode::orientation_sub_callback, this, _1));

        }

    private:
        Interface::datapoint_sub_t          vel_x_sub_;
        Interface::datapoint_sub_t          vel_y_sub_;
        Interface::orientation_sub_t        orientation_sub_;
        Interface::ros_timer_t              position_pub_timer_;
        Interface::position_pub_t           position_pub_;
        Interface::current_state_t          curr_pos_;
        std::shared_ptr<std::vector<float>> currrent_vel_ = nullptr;
        vector<float>                       current_pos_ = vector<float> {0,0};
        vector<float>                       orientation_ = vector<float> {0,0,0};
        float                               current_x_;
        float                               current_y_;
        bool                                start_vel_ = false;
        bool                                start_vel_ = false;

        void initCurrentState()
        {
            auto waitForStartVelocity = std::bind(&DVL::waitForStartVelocity, this);
            std::future<bool> promise = std::async(waitForStartVelocity);
            std::cout << "Waiting for First Velocity Datapoint. \n";
            bool valid = promise.get();
            if (valid) { std::cout << "Got Valid Sensor Info. \n";}
        }

        bool waitForStartVelocity()
        {
            while (this->start_vel_ == nullptr)
            {
                sleep(.1);
            }
            return true;
        }

        void vel_x_sub_callback(const std_msgs::msg::Datapoint::SharedPtr msg)
        {
            // RCLCPP_INFO(this->get_logger(), "X Velocity: %f", msg->data);
            if (start_vel_ == false) {start_vel_ = true;}
            if (current_vel_ == nullptr) {current_vel_ = std::make_shared<std::vector<float>>(2, 0.0f); }
            current_vel_[0] = msg->data;
            this->current_position_[0] += msg->data / (1/UPDATE_RATE);
        }

        void vel_y_sub_callback(const std_msgs::msg::Datapoint::SharedPtr msg)
        {
            // RCLCPP_INFO(this->get_logger(), "Y Velocity: %f", msg->data);
            if (current_vel_ == nullptr) {current_vel_ = std::make_shared<std::vector<float>>(2, 0.0f); } // Initialize velocity after async function
            current_vel_[1] = msg->data;
            this->current_position_[1] += msg->data / (1/UPDATE_RATE);
        }

        void orientation_sub_callback(const scion_types::msg::Orientation::SharedPtr msg)
        {
            this->current_orientation_ = msg->orientation;
        }

        void dead_reckon()
        {
            float total_dist = sqrt(pow(current_vel_[0], 2) + pow(current_vel_[1], 2)) 
            current_pos_[0] += cos(total_dist);
            current_pos_[1] += sin(total_dist);
        }

        void publish_position()
        {
            scion_types::msg::Position position = scion_types::msg::Position();
            position.position = this->current_pos_;
            this->position_pub_->publish(position);
        }
};

  private:
    void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const
    {
        float zed_position_x = (float)msg->pose.position.x;
        float zed_position_y = (float)msg->pose.position.y;
        float zed_position_z = (float)msg->pose.position.z;
        
        auto message = scion_types::msg::Position();
        message.position = std::vector<float>{zed_position_x, zed_position_y, zed_position_z};
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing zed_position_data: " );

    }
    

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DVL>());
  rclcpp::shutdown();
  return 0;
}