#include "rclcpp/rclcpp.hpp"
#include "scion_types/srv/sendframe.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 3) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Usage: Get Depth Data");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("get_depth_client");
  rclcpp::Client<scion_types::srv::SendFrame>::SharedPtr client =
    node->create_client<scion_types::srv::SendFrame>("get_depth");

  auto request = std::make_shared<scion_types::srv::SendFrame::Request>();
  request->can_id = 1234;
  request->can_dlc = 1234;
  request->can_data = "abcd";

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  /*
    int32 can_id
    int8 can_dlc
    char[8] can_data
    ---
    int8 status
  */

  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Depth: %.3f", result.get()->status);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}