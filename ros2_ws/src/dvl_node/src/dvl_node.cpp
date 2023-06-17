#include <vector>
#include <cmath>
#include <memory>
#include <unistd.h>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "control_interface.hpp"
#include "vector_operations.hpp"
#include "scion_types/msg/datapoint.hpp"
#include "scion_types/msg/orientation.hpp"
#include "scion_types/msg/state.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;

#define UPDATE_RATE 50

class DVL : public rclcpp::Node
{
    public:
        DVL()
        : Node("dvl_node")
        {
            vel_x_sub_ = this->create_subscription<scion_types::msg::Datapoint>
            ("dvl_vel_x", 10, std::bind(&DVL::vel_x_sub_callback, this, _1));

            vel_y_sub_ = this->create_subscription<scion_types::msg::Datapoint>
            ("dvl_vel_y", 10, std::bind(&DVL::vel_y_sub_callback, this, _1));

            vel_z_sub_ = this->create_subscription<scion_types::msg::Datapoint>
            ("dvl_vel_z", 10, std::bind(&DVL::vel_z_sub_callback, this, _1));

            orientation_sub_ = this->create_subscription<scion_types::msg::Orientation>
            ("ahrs_orientation_data", 10, std::bind(&DVL::orientation_sub_callback, this, _1));

            position_pub_timer_ = this->create_wall_timer
            (
                std::chrono::milliseconds(50), 
                std::bind(&DVL::publish_position, this)
            );
            
            velocity_pub_timer_ = this->create_wall_timer
            (
                std::chrono::milliseconds(50), 
                std::bind(&DVL::publish_velocity, this)
            );
            
            position_pub_ = this->create_publisher<scion_types::msg::State>("dvl_position_data", 10);

            velocity_pub_ = this->create_publisher<scion_types::msg::State>("dvl_velocity_data", 10);

            start_vel_x_ = nullptr;
            start_vel_y_ = nullptr;
            start_vel_z_ = nullptr;
            start_vel_ =   nullptr;

            auto initFunction = std::bind(&DVL::initCurrentState, this);
            std::thread(initFunction).detach();
        }

    private:
        Interface::datapoint_sub_t          vel_x_sub_;
        Interface::datapoint_sub_t          vel_y_sub_;
        Interface::datapoint_sub_t          vel_z_sub_;
        Interface::orientation_sub_t        orientation_sub_;
        Interface::ros_timer_t              position_pub_timer_;
        Interface::state_pub_t              position_pub_;
        Interface::ros_timer_t              velocity_pub_timer_;
        Interface::state_pub_t              velocity_pub_;
        vector<float>                       currrent_vel_=          vector<float> {0,0,0};
        vector<float>                       current_pos_ =          vector<float> {0,0,0};
        vector<float>                       current_orientation_ =  vector<float> {0,0,0};
        unique_ptr<float>                   start_vel_x_;
        unique_ptr<float>                   start_vel_y_;
        unique_ptr<float>                   start_vel_z_;
        unique_ptr<vector<float>>           start_vel_  ;
        float                               current_x_ =    0.0F;
        float                               current_y_ =    0.0F;
        float                               current_z_ =    0.0F;

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
                if 
                (
                    start_vel_x_ != nullptr &&
                    start_vel_y_ != nullptr &&
                    start_vel_z_ != nullptr
                )
                {
                    start_vel_ = std::make_unique<vector<float>>(initializer_list<float>{*start_vel_x_, *start_vel_y_, *start_vel_z_});
                }
                sleep(.1);
            }
            return true;
        }

        void vel_x_sub_callback(const scion_types::msg::Datapoint::SharedPtr msg)
        {
            if (start_vel_x_ == nullptr) {start_vel_x_  = std::make_unique<float>(msg->data);}
            current_x_ = msg->data;
            this->current_pos_[0] += msg->data * (1/UPDATE_RATE);
        }

        void vel_y_sub_callback(const scion_types::msg::Datapoint::SharedPtr msg)
        {
            if (start_vel_y_ == nullptr) {start_vel_y_  = std::make_unique<float>(msg->data);}
            current_y_ = msg->data;
            this->current_pos_[1] += msg->data * (1/UPDATE_RATE);
        }

        void vel_z_sub_callback(const scion_types::msg::Datapoint::SharedPtr msg)
        {
            if (start_vel_z_ == nullptr) {start_vel_z_  = std::make_unique<float>(msg->data);}
            current_z_ = msg->data;
            this->current_pos_[2] += msg->data * (1/UPDATE_RATE);
        }

        void orientation_sub_callback(const scion_types::msg::Orientation::SharedPtr msg)
        {
            this->current_orientation_ = msg->orientation;
        }

        // void dead_reckon()
        // {
        //     float total_dist = sqrt(pow(current_vel_[0], 2) + pow(current_vel_[1], 2)) 
        //     current_pos_[0] += cos(total_dist);
        //     current_pos_[1] += sin(total_dist);
        // }

        void publish_position()
        {
            scion_types::msg::State position = scion_types::msg::State();
            position.state = this->current_pos_;
            this->position_pub_->publish(position);
        }

        void publish_velocity()
        {
            scion_types::msg::State velocity = scion_types::msg::State();
            velocity.state = this->currrent_vel_;
            this->velocity_pub_->publish(velocity);
        }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DVL>());
  rclcpp::shutdown();
  return 0;
}