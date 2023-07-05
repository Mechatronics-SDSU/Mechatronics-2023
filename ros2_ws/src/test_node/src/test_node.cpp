#include <iterator>
#include <vector>

#include "control_interface.hpp"

using namespace std::placeholders;

class Test : public rclcpp::Node
{
public:
  explicit Test()
  : Node("Test")
  {
    zed_object_pub_ = this->create_publisher<scion_types::msg::VisionObject>("zed_object_data", 10);
    zed_object_timer_ = this->create_wall_timer
                (
                    std::chrono::milliseconds(50), 
                    std::bind(&Test::publishZedObjectTimer, this)
                );

    can_client_ = this->create_client<scion_types::srv::SendFrame>("send_can_raw");
    this->can_timer_ = this->create_wall_timer
    (
      std::chrono::milliseconds(3000), 
      std::bind(&Test::sendFrame, this)
    );
  }

private:
    Interface::ros_sendframe_client_t can_client_;
    Interface::ros_timer_t can_timer_;
    Interface::object_pub_t zed_object_pub_;
    Interface::ros_timer_t zed_object_timer_;

    void publishZedObjectTimer()
    {
      scion_types::msg::VisionObject vision_object = scion_types::msg::VisionObject();
      vision_object.object_name = "gate";
      vision_object.distance = 123.45;
      this->zed_object_pub_->publish(vision_object);
      RCLCPP_INFO(this->get_logger(), "Publishing Test Vision Object" );
    }

    void sendFrame()
    {
        std::vector<unsigned char> kintama = {3, 3, 3, 3, 3, 3, 3, 3};

        sendFrameToYourMomsHouse(10, 8, kintama.data());
    }

    void sendFrameToYourMomsHouse(int32_t can_id, int8_t can_dlc, unsigned char can_data[])
    {
      auto can_request = std::make_shared<scion_types::srv::SendFrame::Request>();
      can_request->can_id = can_id;
      can_request->can_dlc = can_dlc;
      std::copy
      (
          can_data,
          can_data + can_dlc,
          can_request->can_data.begin()
      );
      auto can_future = this->can_client_->async_send_request(can_request);
    }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Test>());
  rclcpp::shutdown();
  return 0;
}


