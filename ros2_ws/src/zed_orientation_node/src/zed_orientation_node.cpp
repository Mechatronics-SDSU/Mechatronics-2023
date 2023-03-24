/* @author Conner Sommerfield - Zix on Discord For Questions
 * Position Node subscribes to Zed node topics and filters out just position to use
 * 
 * Getting zed position
 * ------------------------------------------------------------------------------------------
 * You have to subscribe to zed node topic called /zed2i/zed_node/pose
 *
 *  Then if you do
 *      
 *      ros2 topic type /zed2i/zed_node/pose
 *
 *  It'll show you that it's a geometry_msgs/msg/PoseStamped message type 
 *
 *      ros2 interface show geometry_msgs/msg/PoseStamped
 *
 *  Do this and you'll see it's made up of 
 *
 *  std_msgs/Header header
 *  Pose pose
 *
 * So now we gotta figure out what this shit is
 * 
 *    ros2 interface show geometry_msgs/msg/Pose
 * 
 * Turns out its made of 
 * Point position
 * Quaternion orientation
 * 
 * Orientation is what we need, and it's made up of x, w, y, and z (float64)
 * 
 * --------------------------------------------------------------------------------------------------------
 *  RESET Orientation PROCESS
 *  launch -> subscribe to pose -> watch position
 *  ros2 service call /zed2i/zed_node/reset_pos_tracking std_srvs/srv/Trigger
 *  
 *  You'll see everything go to [0,0,0, 0]
 */


#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "scion_types/msg/orientation.hpp"

using std::placeholders::_1;

void printVector(std::vector<float>& vect)
{
    for (float element : vect)
    {
        std::cout << element << std::endl;
    }
}

class ZedOrientationSubscriber : public rclcpp::Node
{
  public:
    ZedOrientationSubscriber()
    : Node("zed_orientation_node")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("zed2i/zed_node/pose", 10, std::bind(&ZedOrientationSubscriber::topic_callback, this, _1));

        publisher_ = this->create_publisher<scion_types::msg::Orientation>("zed_orientation_data", 10);
    }

  private:
    void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const
    {
        float zed_orientation_x = (float)msg->pose.orientation.x * 180 + 180;
        float zed_orientation_y = (float)msg->pose.orientation.y * 180 + 180;
        float zed_orientation_z = (float)msg->pose.orientation.z * 180 + 180;
        
        auto message = scion_types::msg::Orientation();
        message.orientation = std::vector<float>{zed_orientation_x, zed_orientation_y, zed_orientation_z};
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing zed_orientation_data: " );
        printVector(message.orientation);
    }
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::Publisher<scion_types::msg::Orientation>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZedOrientationSubscriber>());
  rclcpp::shutdown();
  return 0;
}
