#include "current_state_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CurrentStateNode>());
  rclcpp::shutdown();
  return 0;
}