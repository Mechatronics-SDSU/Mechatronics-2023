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

#include "scion_types/action/pid.hpp"
#include "scion_types/msg/idea.hpp"
#include "control_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#define MODE "Mission"

class Brain : public rclcpp::Node
{
    public:
        explicit Brain(): Node("brain_node")
        {
            this->declare_parameter("mode", MODE);
            mode_param = this->get_parameter("mode").get_parameter_value().get<std::string>();

            idea_pub_ = this->create_publisher<scion_types::msg::Idea>("brain_idea_data", 10);
            
            if (mode_param == "Explore")
            {
                decision_timer_ = this->create_wall_timer
                (
                    std::chrono::milliseconds(25), 
                    std::bind(&Brain::make_decision, this)
                );
            }
            if (mode_param == "Mission")
            {   
                sleep(7);
                initSequence(idea_sequence_);
                publishSequence(idea_sequence_);
            }
        }
    private:
        rclcpp::Publisher<scion_types::msg::Idea>::SharedPtr idea_pub_;
        rclcpp::TimerBase::SharedPtr decision_timer_;
        Interface::idea_vector_t idea_sequence_;
        std::string mode_param;

        void make_decision()
        {
            std::cout << "Making A Decision" << std::endl;
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

            // scion_types::msg::Idea idea1 = scion_types::msg::Idea();
            // idea1.code = Idea::TURN;
            // idea1.parameters = std::vector<float>{20};

            // scion_types::msg::Idea idea2 = scion_types::msg::Idea();
            // idea1.code = Idea::MOVE;
            // idea1.parameters = std::vector<float>{0.1};
            
            idea_sequence.push_back(idea1);
            idea_sequence.push_back(idea2);

            idea_sequence.push_back(idea1);
            idea_sequence.push_back(idea2);

            idea_sequence.push_back(idea1);
            idea_sequence.push_back(idea2);

            idea_sequence.push_back(idea1);
            idea_sequence.push_back(idea2);

            idea_sequence.push_back(idea1);
            idea_sequence.push_back(idea2);

            idea_sequence.push_back(idea1);
            idea_sequence.push_back(idea2);

            idea_sequence.push_back(idea1);
            idea_sequence.push_back(idea2);

            idea_sequence.push_back(idea1);
            idea_sequence.push_back(idea2);


        }

        void publishSequence(Interface::idea_vector_t& idea_sequence)
        {   
            using namespace Interface;
            // for (idea_message_t& idea_message : idea_sequence)
            // {
            //     std::cout << idea_message.code << std::endl;
            //     for (float parameter : idea_message.parameters)
            //     {
            //         std::cout << parameter << std::endl;
            //     } 
            // }
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
