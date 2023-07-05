/*
 * @author Conner Sommerfield - Zix on Discord
 * Brain node will send sequence of commands to the PIDs 
 * Commands can be pre-loaded or create with navigation logic
 * 
 * Robot will keep track of a "command queue"
 * This command queue will be abstracted as an vector/queue of Commands
 * Each command will be a function pointer to the action to perform 
 * and any parameters to pass to the function
 * 
 * The main ROS2 purpose of this node is to send a desired state to the
 * PIDs. It's the brain that tells the PIDs where the robot wants to go.
 * 
 * It will do this by taking the next out of the queue
 * 
 * A decision maker will be responsible for loading commands into the queue
 * 
 * PIDs will have to send command completion status for the queue mediator
 * to take out the next command.
 * 
 */

#include <vector>
#include <unistd.h>
#include <iostream>
#include <chrono>

#include "scion_types/action/pid.hpp"
#include "control_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#define MODE "Mission"

using namespace std;

class Brain : public rclcpp::Node
{
    public:
        explicit Brain(): Node("brain_node")
        {
            // this->temp_node_ = rclcpp::Node::make_shared("temp_node");
            idea_pub_ = this->create_publisher<scion_types::msg::Idea>("brain_idea_data", 10);
            can_client_ = this->create_client<scion_types::srv::SendFrame>("send_can_raw");
            // object_sub_ = this->create_subscription<scion_types::msg::VisionObject>
            // ("zed_object_data", 10, [this](const scion_types::msg::VisionObject::SharedPtr msg) {
            //          RCLCPP_INFO(this->get_logger(), "Publishing Idea" );
            // });
         
            this->declare_parameter("mode", MODE);
            mode_param_ = this->get_parameter("mode").get_parameter_value().get<std::string>();
            // if (mode_param_ == "Explore")
            // {
            //     decision_timer_ = this->create_wall_timer
            //     (
            //         std::chrono::milliseconds(25), 
            //         std::bind(&Brain::make_decision, this)
            //     );
            // }
            // if (mode_param_ == "Mission")
            // {   

            // }
            moveUntil(10, &Brain::gateSeen, this->gate_seen_);
            // gateSeen();
        }
    private:
        Interface::idea_pub_t                       idea_pub_;
        Interface::ros_timer_t                      decision_timer_;
        Interface::idea_vector_t                    idea_sequence_;
        Interface::object_sub_t                     object_sub_;
        std::string                                 mode_param_;
        Interface::ros_sendframe_client_t           can_client_;
        bool                                        gate_seen_ = false; 

        void moveUntil(int power, bool (Brain::*condition)(), bool& condition_global)
        {
            // vector<unsigned char> motor_power{power, 0, power, 0, power, 0, power, 0};
            auto condition_met = std::bind(condition, this);
            std::thread(condition_met).detach();

            while(!condition_global) //this->gateSeen()
            {
                RCLCPP_INFO(this->get_logger(), "sent CAN Command");
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
            condition_global = false;
            exit(1);
        }

        bool gateSeen()
        {
            std::promise<bool> gate_seen;
            std::shared_future<bool> future  = gate_seen.get_future();
            Interface::node_t temp_node = rclcpp::Node::make_shared("zed_object_subscriber");;
            Interface::object_sub_t object_sub = temp_node->create_subscription<scion_types::msg::VisionObject>
            ("zed_object_data", 10, [&temp_node, &gate_seen](const scion_types::msg::VisionObject::SharedPtr msg) {
                    if (msg->object_name == "gate") {
                        gate_seen.set_value(true);
                        RCLCPP_INFO(temp_node->get_logger(), "Gate seen");
                    }
            });
            rclcpp::spin_until_future_complete(temp_node, future);
            this->gate_seen_ = true;
            return true;
        }

        void initSequence(Interface::idea_vector_t& idea_sequence)
        {
            using namespace Interface;
            scion_types::msg::Idea idea1 = scion_types::msg::Idea();
            idea1.code = Idea::RELATIVE_POINT;
            idea1.parameters = std::vector<float>{0.0F,-0.3F};

            scion_types::msg::Idea idea2 = scion_types::msg::Idea();
            idea2.code = Idea::RELATIVE_POINT;
            idea2.parameters = std::vector<float>{0.0F,-0.3F};
        }

        void publishSequence(Interface::idea_vector_t& idea_sequence)
        {   
            using namespace Interface;
            for (idea_message_t& idea_message : idea_sequence)
            {
                sleep(1);
                this->idea_pub_->publish(idea_message);
            }
            RCLCPP_INFO(this->get_logger(), "Publishing Idea" );
        }

}; // class Brain


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Brain>());
  rclcpp::shutdown();
  return 0;
}
