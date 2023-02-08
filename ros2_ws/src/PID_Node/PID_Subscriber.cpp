#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "scion_types/msg/string.hpp"
using std::placeholders::_1;

class PID_Subscriber : public rclcpp::Node
{
  public:
    PID_Subscriber()
    : Node("ms5837_depth")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "ms5837_depth", 10, std::bind(&PID_Subscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::Float32::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Recieved Depth Ino: '%.3f'", msg->data);
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PID_Subscriber>());
  rclcpp::shutdown();
  return 0;
}