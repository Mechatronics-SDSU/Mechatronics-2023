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
            idea_pub_ = this->create_publisher<scion_types::msg::Idea>("brain_idea_data", 10);
            // object_sub_ = this->create_subscription<scion_types::msg::VisionObject>
            // ("zed_object_data", 10, [this](const scion_types::msg::VisionObject::SharedPtr msg) {
            //          RCLCPP_INFO(this->get_logger(), "Publishing Idea" );
            // });
         
            this->declare_parameter("mode", MODE);
            mode_param = this->get_parameter("mode").get_parameter_value().get<std::string>();
            // if (mode_param == "Explore")
            // {
            //     decision_timer_ = this->create_wall_timer
            //     (
            //         std::chrono::milliseconds(25), 
            //         std::bind(&Brain::make_decision, this)
            //     );
            // }
            // if (mode_param == "Mission")
            // {   
            // }

            seeGate();
            
        }
    private:
        Interface::idea_pub_t idea_pub_;
        Interface::ros_timer_t decision_timer_;
        Interface::idea_vector_t idea_sequence_;
        std::string mode_param;
        Interface::object_sub_t object_sub_;

        void moveUntil(int power, Interface::predicate_function condition)
        {
            
        }

        bool seeGate()
        {
            bool gateSeen = false;
            
            // Interface::object_sub_t object_sub;
            object_sub_ = this->create_subscription<scion_types::msg::VisionObject>
            ("zed_object_data", 10, [this](const scion_types::msg::VisionObject::SharedPtr msg) {
                // if ("a" == "a") {
                    RCLCPP_INFO(this->get_logger(), "Publishing Idea" );
                // }
            });

            while (!gateSeen)
            {
                cout << "No gate" << endl;
            }
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
