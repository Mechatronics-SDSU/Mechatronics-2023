#include "a50_node.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<A50Node>());
  rclcpp::shutdown();
  return 0;
}