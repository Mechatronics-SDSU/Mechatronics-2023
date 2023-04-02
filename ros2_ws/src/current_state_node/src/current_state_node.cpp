#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "scion_types/msg/desired_state.hpp"

/* Subscribe to all relevant sensor information and consolidate it for the PID Node to subscribe to */

class CurrentStateNode : public rclcpp::Node
{
  public:
    CurrentStateNode()
    : Node("current_state_node")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("zed2i/zed_node/pose", 10, std::bind(&CurrentStateNode::topic_callback, this, _1));

        publisher_ = this->create_publisher<scion_types::msg::Position>("current_state_data", 10);
    }

  private:
    void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const
    {
        
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::Publisher<scion_types::msg::Position>::SharedPtr publisher_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CurrentStateNode>());
  rclcpp::shutdown();
  return 0;
}
