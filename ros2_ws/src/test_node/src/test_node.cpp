#include <iterator>
#include <vector>
#include <chrono>

#include "control_interface.hpp"

using namespace std::placeholders;
using namespace std;

class Test : public rclcpp::Node
{
public:
  explicit Test()
  : Node("Test")
  {
    // zed_object_pub_ = this->create_publisher<scion_types::msg::VisionObject>("zed_object_data", 10);
    // zed_object_timer_ = this->create_wall_timer
    //             (
    //                 std::chrono::milliseconds(50), 
    //                 std::bind(&Test::publishZedObjectTimer, this)
    //             );

    zed_vision_pub_ = this->create_publisher<scion_types::msg::ZedObject>("zed_vision_data", 10);
    zed_vision_timer_ = this->create_wall_timer
                (
                    std::chrono::milliseconds(500), 
                    std::bind(&Test::publishZedVisionTimer, this)
                );

    can_client_ = this->create_client<scion_types::srv::SendFrame>("send_can_raw");
    this->can_timer_ = this->create_wall_timer
    (
      std::chrono::milliseconds(3000), 
      std::bind(&Test::sendFrame, this)
    );

    idea_sub_ = this->create_subscription<scion_types::msg::Idea>
    (
      "brain_idea_data",
       10,
       std::bind(&Test::testIdeaCallback, this, _1)
    );

    command_queue_is_empty_sub_ = this->create_subscription<std_msgs::msg::Int32>
    (
      "is_command_queue_empty",
       10,
       std::bind(&Test::testCommandQueueEmptyCallback, this, _1)
    );
  }

private:
    Interface::ros_sendframe_client_t can_client_;
    Interface::ros_timer_t can_timer_;
    Interface::object_pub_t zed_object_pub_;
    Interface::vision_pub_t zed_vision_pub_;
    Interface::ros_timer_t zed_object_timer_;
    Interface::ros_timer_t zed_vision_timer_;
    Interface::idea_sub_t  idea_sub_;
    Interface::int_sub_t command_queue_is_empty_sub_;
    int count_ = 0;

    void testIdeaCallback(scion_types::msg::Idea::SharedPtr idea)
    {
        // RCLCPP_INFO(this->get_logger(), "Recieved idea %d for degree %f", idea->code);
    }

    void testCommandQueueEmptyCallback(std_msgs::msg::Int32::SharedPtr is_command_queue_empty) 
    {
        if(is_command_queue_empty->data) 
        {
            RCLCPP_INFO(this->get_logger(), "Command queue empty" );
        }
        else 
        {
            RCLCPP_INFO(this->get_logger(), "Command queue not empty" );
        }
    }

    void publishZedObjectTimer()
    {
      scion_types::msg::VisionObject vision_object = scion_types::msg::VisionObject();
      vision_object.object_name = "gate";
      vision_object.distance = 123.45;
      this->zed_object_pub_->publish(vision_object);
      RCLCPP_INFO(this->get_logger(), "Publishing Test Vision Object" );
    }

     void publishZedVisionTimer()
    {
      scion_types::msg::ZedObject vision_object = scion_types::msg::ZedObject();
      std::array<scion_types::msg::Keypoint2Di, 4> bounding_box_from_zed;

      scion_types::msg::Keypoint2Di keypoint = scion_types::msg::Keypoint2Di();
      keypoint.kp[0] = count_;
      keypoint.kp[1] = count_;
      bounding_box_from_zed[0] = keypoint;
      bounding_box_from_zed[1] = keypoint;
      bounding_box_from_zed[2] = keypoint;
      bounding_box_from_zed[3] = keypoint;

      vision_object.corners =  bounding_box_from_zed;
      this->zed_vision_pub_->publish(vision_object);
      count_ ++;
      RCLCPP_INFO(this->get_logger(), "Publishing Test Vision Bounding Box with point %d", count_);
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


