#include "scion_types/srv/send_frame.hpp"
#include <iterator>
#include "rclcpp/rclcpp.hpp"
#include <vector>
using namespace std::placeholders;

class Test : public rclcpp::Node
{
public:
  explicit Test()
  : Node("Test")
  {
    can_client_ = this->create_client<scion_types::srv::SendFrame>("send_can_raw");

    this->timer_ = this->create_wall_timer
    (
      std::chrono::milliseconds(100), 
      std::bind(&Test::sendFrame, this)
    );
  }

private:
  rclcpp::Client<scion_types::srv::SendFrame>::SharedPtr can_client_;
  rclcpp::TimerBase::SharedPtr timer_;

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


