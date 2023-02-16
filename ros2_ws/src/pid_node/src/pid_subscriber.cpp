#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "scion_types/msg/position.hpp"

using std::placeholders::_1;

class PID_Subscriber : public rclcpp::Node
{
  public:
    PID_Subscriber()
    : Node("ms5837_depth")
    {
      depth_ = this->create_subscription<std_msgs::msg::Float32>(
      "ms5837_depth", 10, std::bind(&PID_Subscriber::depth_callback, this, _1));
      orientation_ = this->create_subscription<scion_types::msg::Orientation>(
      "ahrs", 10, std::bind(&PID_Subscriber::orientation_callback, this, _1));
    }

  private:
    void depth_callback(const std_msgs::msg::Float32::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Recieved Depth Ino: '%.3f'", msg->data);
    }
    void orientation_callback(const std_msgs::msg::Float32::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Recieved Depth Ino: '%.3f'", msg->data);
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr depth_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr orientation_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PID_Subscriber>());
  rclcpp::shutdown();
  return 0;
}