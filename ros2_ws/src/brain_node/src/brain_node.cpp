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

#define SMOOTH_TURN_POWER 15
#define SUBMERGE_DISTANCE 1.5f

class Brain : public rclcpp::Node
{
    typedef bool (Brain::*condition_t)();
    typedef void (action_t)(int);

    public:
        explicit Brain(): Node("brain_node")
        {
            idea_pub_ = this->create_publisher<scion_types::msg::Idea>("brain_idea_data", 10);
            can_client_ = this->create_client<scion_types::srv::SendFrame>("send_can_raw");
            this->performMission();
        }
    private:
        Interface::idea_pub_t                       idea_pub_;
        Interface::ros_timer_t                      decision_timer_;
        Interface::idea_vector_t                    idea_sequence_;
        Interface::object_sub_t                     object_sub_;
        std::string                                 mode_param_;
        Interface::ros_sendframe_client_t           can_client_;
        bool                                        gate_seen_ = false; 
        
        void doUntil(action_t action, condition_t condition, bool& condition_global, int parameter)
        {
            auto condition_met = std::bind(condition, this);
            std::thread(condition_met).detach();

            while(!condition_global) //this->gateSeen()
            {
                (action)(parameter);
            }
            condition_global = false;
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

        void stop()
        {
            scion_types::msg::Idea idea = scion_types::msg::Idea();
            idea.code = Interface::Idea::STOP;
            idea_pub_->publish(idea);
        }

        void go(float degree)
        {
            scion_types::msg::Idea idea = scion_types::msg::Idea();
            idea.code = Interface::Idea::GO;
            idea.parameters = std::vector<float>{degree};
            idea_pub_->publish(idea);
        }

        void spin(float degree)
        {
            scion_types::msg::Idea idea = scion_types::msg::Idea();
            idea.code = Interface::Idea::SPIN;
            idea.parameters = std::vector<float>{degree};
            idea_pub_->publish(idea);
        }

        void turn(float degree)
        {
            scion_types::msg::Idea idea = scion_types::msg::Idea();
            idea.code = Interface::Idea::TURN;
            idea.parameters = std::vector<float>{degree};
            idea_pub_->publish(idea);
        }

        void pitch(float degree)
        {
            scion_types::msg::Idea idea = scion_types::msg::Idea();
            idea.code = Interface::Idea::PITCH;
            idea.parameters = std::vector<float>{degree};
            idea_pub_->publish(idea);
        }

        void roll(float degree)
        {
            scion_types::msg::Idea idea = scion_types::msg::Idea();
            idea.code = Interface::Idea::ROLL;
            idea.parameters = std::vector<float>{degree};
            idea_pub_->publish(idea);
        }

        void moveForward(float degree)
        {
            scion_types::msg::Idea idea = scion_types::msg::Idea();
            idea.code = Interface::Idea::MOVE;
            idea.parameters = std::vector<float>{degree};
            idea_pub_->publish(idea);
            RCLCPP_INFO(this->get_logger(), "Moving Forward %f meters", degree);
        }

        void translate(float degree)
        {
            scion_types::msg::Idea idea = scion_types::msg::Idea();
            idea.code = Interface::Idea::TRANSLATE;
            idea.parameters = std::vector<float>{degree};
            idea_pub_->publish(idea);
        }

        void levitate(float degree)
        {
            scion_types::msg::Idea idea = scion_types::msg::Idea();
            idea.code = Interface::Idea::LEVITATE;
            idea.parameters = std::vector<float>{degree};
            idea_pub_->publish(idea);
        }

        static void keepTurning(int power)
        {
            auto logger = rclcpp::get_logger("my_logger");
            RCLCPP_INFO(logger, "turning with power of %d", power);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        float getDistanceFromCamera(string object)
        {
            float distance;
            std::promise<bool> gate_seen;
            std::shared_future<bool> future  = gate_seen.get_future();
            Interface::node_t temp_node = rclcpp::Node::make_shared("zed_object_subscriber");;
            Interface::object_sub_t object_sub = temp_node->create_subscription<scion_types::msg::VisionObject>
            ("zed_object_data", 10, [&temp_node, &gate_seen, &object, &distance](const scion_types::msg::VisionObject::SharedPtr msg) {
                    if (msg->object_name == object) {
                        gate_seen.set_value(true);
                        distance = msg->distance;
                    }
            });
            rclcpp::spin_until_future_complete(temp_node, future);
            return distance;
        }

        ////////////////////////////////////////////////////////////////////////////////
        //                                  MISSION                                   //
        ////////////////////////////////////////////////////////////////////////////////
v
        void performMission()
        {
            doUntil(&keepTurning, &Brain::gateSeen, this->gate_seen_, SMOOTH_TURN_POWER);
            levitate(SUBMERGE_DISTANCE);
            moveForward(this->getDistanceFromCamera("gate"));
            exit(1);
        }


}; // class Brain


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Brain>());
  rclcpp::shutdown();
  return 0;
}



/*          doUntil(&Brain::gateSeen, [](int power)
            {
                auto logger = rclcpp::get_logger("my_logger");
                RCLCPP_INFO(logger, "sent CAN Command of power %d", power);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            },  this->gate_seen_, 20); */

/* 
    void moveUntil(int power, condition_t condition, bool& condition_global)
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
    } */


/* void initSequence(Interface::idea_vector_t& idea_sequence)
    {
        using namespace Interface;
        scion_types::msg::Idea idea1 = scion_types::msg::Idea();
        idea1.code = Idea::RELATIVE_POINT;
        idea1.parameters = std::vector<float>{0.0F,-0.3F};

        scion_types::msg::Idea idea2 = scion_types::msg::Idea();
        idea2.code = Idea::RELATIVE_POINT;
        idea2.parameters = std::vector<float>{0.0F,-0.3F};
    } */


/* void publishSequence(Interface::idea_vector_t& idea_sequence)
    {   
            using namespace Interface;
            for (idea_message_t& idea_message : idea_sequence)
            {
                sleep(1);
                this->idea_pub_->publish(idea_message);
            }
            RCLCPP_INFO(this->get_logger(), "Publishing Idea" );
    } */