#include "rclcpp/rclcpp.hpp"
#include "brain_node.hpp"
#include "control_interface.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Brain>());
  rclcpp::shutdown();
  return 0;
}