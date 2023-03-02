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
 * position is what we need, and it's made up of x, y, and z (float64)
 * 
 * --------------------------------------------------------------------------------------------------------
 *  RESET POSITION PROCESS
 *  launch -> subscribe to pose -> watch position
 *  ros2 service call /zed2i/zed_node/reset_pos_tracking std_srvs/srv/Trigger
 *  
 *  You'll see everything go to [0,0,0]
 */


#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "scion_types/msg/position.hpp"

using std::placeholders::_1;

void printVector(std::vector<double> vect)
{
    for (double element : vect)
    {
        std::cout << element << std::endl;
    }
}

class ZedPositionSubscriber : public rclcpp::Node
{
  public:
    ZedPositionSubscriber()
    : Node("zed_position_node")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("zed2i/zed_node/pose", 10, std::bind(&ZedPositionSubscriber::topic_callback, this, _1));

        publisher_ = this->create_publisher<scion_types::msg::Position>("zed_position_data", 10);
    }

  private:
    void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const
    {
        float zed_position_x = (float)msg->pose.position.x;
        float zed_position_y = (float)msg->pose.position.y;
        float zed_position_z = (float)msg->pose.position.z;
        
        auto message = scion_types::msg::Position();
        message.position = std::vector<float>{zed_position_x, zed_position_y, zed_position_z};
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing zed_position_data: " );

    }
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::Publisher<scion_types::msg::Position>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZedPositionSubscriber>());
  rclcpp::shutdown();
  return 0;
}
