#include "mediator_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mediator>());
  rclcpp::shutdown();
  return 0;
}