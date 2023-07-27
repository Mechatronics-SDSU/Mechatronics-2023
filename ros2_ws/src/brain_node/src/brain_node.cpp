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
#include <cmath>
#include "brain_node.hpp"
#include "scion_types/action/pid.hpp"
#include "control_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "vector_operations.hpp"
#include "filter.hpp"

Brain::Brain() : Node("brain_node")
{
    idea_pub_ = this->create_publisher<scion_types::msg::Idea>("brain_idea_data", 10);
    can_client_ = this->create_client<scion_types::srv::SendFrame>("send_can_raw");
    pid_ready_service_ = this->create_service<std_srvs::srv::Trigger>("pid_ready", std::bind(&Brain::ready, this, _1, _2));
    int_ = 2;
}

////////////////////////////////////////////////////////////////////////////////
//                               INIT MISSION                                 //
////////////////////////////////////////////////////////////////////////////////

void Brain::ready(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    this->performMission();
}

////////////////////////////////////////////////////////////////////////////////
//                               ASYNC FUNCTIONS                              //
////////////////////////////////////////////////////////////////////////////////

void Brain::doUntil(action_t action, condition_t condition, cleanup_t cleanup, bool& condition_global, float parameter)
{
    auto condition_met = std::bind(condition, this);
    std::thread(condition_met).detach();

    while(!condition_global) //this->gateSeen()
    {
        (this->*action)(parameter);
    }
    condition_global = false;
    (this->*cleanup)();
}

bool Brain::gateSeen()
{
    std::promise<bool> gate_seen;
    std::shared_future<bool> future  = gate_seen.get_future();
    Interface::node_t temp_node = rclcpp::Node::make_shared("zed_object_subscriber");;
    Interface::object_sub_t object_sub = temp_node->create_subscription<scion_types::msg::VisionObject>
    ("zed_object_data", 10, [&temp_node, &gate_seen](const scion_types::msg::VisionObject::SharedPtr msg) {
            if (msg->object_name == "Underwater-Gate") {
                gate_seen.set_value(true);
                RCLCPP_INFO(temp_node->get_logger(), "Gate seen");
            }
    });
    rclcpp::spin_until_future_complete(temp_node, future);
    this->gate_seen_ = true;
    return true;
}

float Brain::getDistanceFromCamera(string object)
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

unique_ptr<Filter> Brain::populateFilterBuffer(int object_identifier)
{
    size_t data_streams_num = 1;
    string coeff_file = "/home/mechatronics/master/ros2_ws/src/brain_node/coefficients.txt";
    unique_ptr<Filter> moving_average_filter = std::make_unique<Filter>(data_streams_num, coeff_file);

    std::promise<bool> buffer_filled;
    std::shared_future<bool> future  = buffer_filled.get_future();
    Interface::node_t temp_node = rclcpp::Node::make_shared("zed_vision_subscriber");
    Interface::vision_sub_t object_sub = temp_node->create_subscription<scion_types::msg::ZedObject>
    ("zed_vision_data", 10, [this, &temp_node, &buffer_filled, &moving_average_filter, &object_identifier](const scion_types::msg::ZedObject::SharedPtr msg) {
            if (msg->label_id != object_identifier) {return;}
            unique_ptr<vector<vector<uint32_t>>>ros_bounding_box = zed_to_ros_bounding_box(msg->corners);
            vector<uint32_t> bounding_box_midpoint = findMidpoint(*ros_bounding_box);
            moving_average_filter->smooth(moving_average_filter->data_streams[0], (float)bounding_box_midpoint[0]);
            if (moving_average_filter->data_streams[0][10] != 0)
            {
                buffer_filled.set_value(true);              // jump out of async spin
            }
    });
    rclcpp::spin_until_future_complete(temp_node, future);
    return moving_average_filter;
}

void Brain::centerRobot(int object_identifier)
{
    unique_ptr<Filter> moving_average_filter = populateFilterBuffer(object_identifier);
    RCLCPP_INFO(this->get_logger(), "Filter Buffer Filled");
    vector<uint32_t> camera_frame_midpoint {MID_X_PIXEL, MID_Y_PIXEL};

    std::promise<bool> robot_centered;
    std::shared_future<bool> future  = robot_centered.get_future();
    Interface::node_t temp_node = rclcpp::Node::make_shared("zed_vision_subscriber");
    Interface::vision_sub_t object_sub = temp_node->create_subscription<scion_types::msg::ZedObject>
    ("zed_vision_data", 10, [this, &temp_node, &camera_frame_midpoint, &robot_centered, &moving_average_filter, &object_identifier](const scion_types::msg::ZedObject::SharedPtr msg) {
            if (msg->label_id != object_identifier) {return;}
            unique_ptr<vector<vector<uint32_t>>>ros_bounding_box = zed_to_ros_bounding_box(msg->corners);
            vector<uint32_t> bounding_box_midpoint = findMidpoint(*ros_bounding_box);
            float filtered_bounding_box_midpoint = moving_average_filter->smooth(moving_average_filter->data_streams[0], (float)bounding_box_midpoint[0]);
            
            if (areEqual(bounding_box_midpoint, camera_frame_midpoint)) {robot_centered.set_value(true);}
            else {
                if (isCommandQueueEmpty())
                {
                    RCLCPP_INFO(this->get_logger(), "Looking at bounding box with value %d", (*ros_bounding_box)[0][0]);
                    this->adjustToCenter(bounding_box_midpoint, camera_frame_midpoint);
                }
            }
    });
    rclcpp::spin_until_future_complete(temp_node, future);
}

std::unique_ptr<vector<vector<uint32_t>>> Brain::zed_to_ros_bounding_box(std::array<scion_types::msg::Keypoint2Di, 4>& zed_bounding_box)
{
    vector<vector<uint32_t>> ros_bounding_box;                   // this is to convert from ros object to vector object
    for (int i = 0; i < NUM_CORNERS; i++)
    {
        vector<uint32_t> corner_point {((zed_bounding_box[i]).kp)[i], ((zed_bounding_box[i]).kp)[i+1]};
        ros_bounding_box.push_back(corner_point);
    }
    return std::make_unique<vector<vector<uint32_t>>>(ros_bounding_box);
}

vector<uint32_t> Brain::findMidpoint(vector<vector<uint32_t>>& bounding_box)
{
    vector<uint32_t> cornerUL = bounding_box[0];
    vector<uint32_t> cornerUR = bounding_box[1];
    vector<uint32_t> cornerBL = bounding_box[2];
    uint32_t midpoint_x = ((cornerUR + cornerUL)/((uint32_t)2))[0];
    uint32_t midpoint_y = (720 - ((cornerUL + cornerBL)/((uint32_t)2))[1]);
    vector<uint32_t> midpoint {midpoint_x, midpoint_y};
    return midpoint;
}

bool Brain::areEqual(vector<uint32_t> point_a, vector<uint32_t> point_b)
{
    return (abs((int)((point_a - point_b))[0]) < PIXEL_ERROR_THRESHOLD);
}

void Brain::adjustToCenter(vector<uint32_t> bounding_box_midpoint, vector<uint32_t> camera_frame_midpoint)
{   
    bool bounding_box_is_to_the_right_of_center_pixel = bounding_box_midpoint[0] > camera_frame_midpoint[0];
    if (bounding_box_is_to_the_right_of_center_pixel) {this->turn(TO_THE_RIGHT);}
    else {this->turn(TO_THE_LEFT);}
}

void Brain::waitForCommandQueueEmpty()
{
    std::promise<bool> command_queue_empty;
    std::shared_future<bool> future  = command_queue_empty.get_future();
    Interface::node_t temp_node = rclcpp::Node::make_shared("command_queue_empty");
    Interface::int_sub_t command_queue_empty_sub = temp_node->create_subscription<std_msgs::msg::Int32>
    ("is_command_queue_empty", 10, [this, &temp_node, &command_queue_empty](const std_msgs::msg::Int32::SharedPtr msg) {
            if (msg->data) {command_queue_empty.set_value(true);}
    });
    rclcpp::spin_until_future_complete(temp_node, future);
}

bool Brain::isCommandQueueEmpty()
{
    bool isEmpty;
    std::promise<bool> command_queue_empty;
    std::shared_future<bool> future  = command_queue_empty.get_future();
    Interface::node_t temp_node = rclcpp::Node::make_shared("command_queue_empty");
    Interface::int_sub_t command_queue_empty_sub = temp_node->create_subscription<std_msgs::msg::Int32>
    ("is_command_queue_empty", 10, [this, &temp_node, &isEmpty, &command_queue_empty](const std_msgs::msg::Int32::SharedPtr msg) {
            command_queue_empty.set_value(msg->data);
            isEmpty = msg->data;
    });
    rclcpp::spin_until_future_complete(temp_node, future);
    return isEmpty;
}

////////////////////////////////////////////////////////////////////////////////
//                               MOVEMENT IDEAS                               //
////////////////////////////////////////////////////////////////////////////////


void Brain::stop()
{
    scion_types::msg::Idea idea = scion_types::msg::Idea();
    idea.code = Interface::Idea::STOP;
    idea_pub_->publish(idea);
}

void Brain::turn(float degree)
{
    scion_types::msg::Idea idea = scion_types::msg::Idea();
    idea.code = Interface::Idea::TURN;
    idea.parameters = std::vector<float>{degree};
    idea_pub_->publish(idea);
    RCLCPP_INFO(this->get_logger(), "Turning %f Degrees", degree);
}

void Brain::pitch(float degree)
{
    scion_types::msg::Idea idea = scion_types::msg::Idea();
    idea.code = Interface::Idea::PITCH;
    idea.parameters = std::vector<float>{degree};
    idea_pub_->publish(idea);
}

void Brain::roll(float degree)
{
    scion_types::msg::Idea idea = scion_types::msg::Idea();
    idea.code = Interface::Idea::ROLL;
    idea.parameters = std::vector<float>{degree};
    idea_pub_->publish(idea);
}

void Brain::moveForward(float degree)
{
    scion_types::msg::Idea idea = scion_types::msg::Idea();
    idea.code = Interface::Idea::MOVE;
    idea.parameters = std::vector<float>{degree};
    idea_pub_->publish(idea);
}

void Brain::translate(float degree)
{
    scion_types::msg::Idea idea = scion_types::msg::Idea();
    idea.code = Interface::Idea::TRANSLATE;
    idea.parameters = std::vector<float>{degree};
    idea_pub_->publish(idea);
}

void Brain::levitate(float degree)
{
    scion_types::msg::Idea idea = scion_types::msg::Idea();
    idea.code = Interface::Idea::LEVITATE;
    idea.parameters = std::vector<float>{degree};
    idea_pub_->publish(idea);
}

void Brain::keepTurning(float power)
{
    this->turn(power);
    RCLCPP_INFO(this->get_logger(), "Turning");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void Brain::keepMoving(float power)
{
    this->moveForward(power);
    RCLCPP_INFO(this->get_logger(), "Moving Forward");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

////////////////////////////////////////////////////////////////////////////////
//                                  MISSION                                   //
////////////////////////////////////////////////////////////////////////////////

void Brain::performMission()
{
    this->centerRobot(0);       // 0 is identifier of gate
    doUntil(&Brain::keepTurning, &Brain::gateSeen, &Brain::stop, this->gate_seen_, SMOOTH_TURN_DEGREE);
    this->centerRobot(0);
    levitate(SUBMERGE_DISTANCE);
    moveForward(this->getDistanceFromCamera("Full- Gate")/2);
    this->centerRobot(0);
    exit(0);
} 















































        ////////////////////////////////////////////////////////////////////////////////
        //                                  ARCHIVE                                   //
        ////////////////////////////////////////////////////////////////////////////////

        // auto logger = rclcpp::get_logger("my_logger");
        // RCLCPP_INFO(logger, "turning with power of %d", power);



/*      doUntil(&Brain::gateSeen, [](int power)
        {
            auto logger = rclcpp::get_logger("my_logger");
            RCLCPP_INFO(logger, "sent CAN Command of power %d", power);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        },  this->gate_seen_, 20); */

/* 
    void Brain::moveUntil(int power, condition_t condition, bool& condition_global)
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