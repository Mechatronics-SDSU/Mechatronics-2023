#include <vector>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>

#include "filter.hpp"
#include "scion_types/msg/keypoint2_di.hpp"
#include "control_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vector_operations.hpp"

#define MID_X_PIXEL 640
#define MID_Y_PIXEL 360

std::unique_ptr<std::vector<std::vector<uint32_t>>> zed_to_ros_bounding_box(std::array<scion_types::msg::Keypoint2Di, 4>& zed_bounding_box)
{
    std::vector<std::vector<uint32_t>> ros_bounding_box;                   // this is to convert from ros object to std::vector object
    for (int i = 0; i < 4; i++)
    {
        std::vector<uint32_t> corner_point {((zed_bounding_box[i]).kp)[0], ((zed_bounding_box[i]).kp)[1]};
        ros_bounding_box.push_back(corner_point);
    }
    return std::make_unique<std::vector<std::vector<uint32_t>>>(ros_bounding_box);
}

vector<uint32_t> findMidpoint(vector<vector<uint32_t>>& bounding_box)
{
    vector<uint32_t> cornerUL = bounding_box[0];
    vector<uint32_t> cornerUR = bounding_box[1];
    vector<uint32_t> cornerBL = bounding_box[2];
    uint32_t midpoint_x = ((cornerUR + cornerUL)/((uint32_t)2))[0];
    uint32_t midpoint_y = (720 - ((cornerUL + cornerBL)/((uint32_t)2))[1]);
    vector<uint32_t> midpoint {midpoint_x, midpoint_y};
    return midpoint;
}

unique_ptr<Filter> populateFilterBuffer(int object_identifier)
{
    size_t data_streams_num = 1;
    string coeff_file = "/home/mechatronics/master/ros2_ws/src/brain_node/coefficients.txt";
    unique_ptr<Filter> moving_average_filter = std::make_unique<Filter>(data_streams_num, coeff_file);

    std::promise<bool> buffer_filled;
    std::shared_future<bool> future  = buffer_filled.get_future();
    Interface::node_t temp_node = rclcpp::Node::make_shared("zed_vision_subscriber");
    Interface::vision_sub_t object_sub = temp_node->create_subscription<scion_types::msg::ZedObject>
    ("zed_vision_data", 10, [&temp_node, &buffer_filled, &moving_average_filter, &object_identifier](const scion_types::msg::ZedObject::SharedPtr msg) {
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

void centerRobot(int object_identifier)
{
    unique_ptr<Filter> moving_average_filter = populateFilterBuffer(object_identifier);
    vector<uint32_t> camera_frame_midpoint {MID_X_PIXEL, MID_Y_PIXEL};

    std::promise<bool> robot_centered;
    std::shared_future<bool> future  = robot_centered.get_future();
    Interface::node_t temp_node = rclcpp::Node::make_shared("zed_vision_subscriber");
    Interface::vision_sub_t object_sub = temp_node->create_subscription<scion_types::msg::ZedObject>
    ("zed_vision_data", 10, [temp_node, &camera_frame_midpoint, &robot_centered, &moving_average_filter, &object_identifier](const scion_types::msg::ZedObject::SharedPtr msg) {
            if (msg->label_id != object_identifier) {return;}
            unique_ptr<vector<vector<uint32_t>>>ros_bounding_box = zed_to_ros_bounding_box(msg->corners);
            vector<uint32_t> bounding_box_midpoint = findMidpoint(*ros_bounding_box);
            float filtered_bounding_box_midpoint = moving_average_filter->smooth(moving_average_filter->data_streams[0], (float)bounding_box_midpoint[0]);
            if ((filtered_bounding_box_midpoint - camera_frame_midpoint[0]) < .05) 
            {
                robot_centered.set_value(true);
                RCLCPP_INFO(temp_node->get_logger(), "Looking at bounding box with value %f", filtered_bounding_box_midpoint);
                RCLCPP_INFO(temp_node->get_logger(), "Test Passed");
                return;
            }
            else
            {
                RCLCPP_INFO(temp_node->get_logger(), "Looking at bounding box with value %f", filtered_bounding_box_midpoint);
                RCLCPP_INFO(temp_node->get_logger(), "Test Failed");
                return;
            }
    });
    rclcpp::spin_until_future_complete(temp_node, future);
}

void centerRobotRight(int object_identifier)
{
    unique_ptr<Filter> moving_average_filter = populateFilterBuffer(object_identifier);
    vector<uint32_t> camera_frame_midpoint {MID_X_PIXEL, MID_Y_PIXEL};

    std::promise<bool> robot_centered;
    std::shared_future<bool> future  = robot_centered.get_future();
    Interface::node_t temp_node = rclcpp::Node::make_shared("zed_vision_subscriber");
    Interface::vision_sub_t object_sub = temp_node->create_subscription<scion_types::msg::ZedObject>
    ("zed_vision_data", 10, [temp_node, &camera_frame_midpoint, &robot_centered, &moving_average_filter, &object_identifier](const scion_types::msg::ZedObject::SharedPtr msg) {
            if (msg->label_id != object_identifier) {return;}
            unique_ptr<vector<vector<uint32_t>>>ros_bounding_box = zed_to_ros_bounding_box(msg->corners);
            vector<uint32_t> bounding_box_midpoint = findMidpoint(*ros_bounding_box);
            float filtered_bounding_box_midpoint = moving_average_filter->smooth(moving_average_filter->data_streams[0], (float)bounding_box_midpoint[0]);
            if ((filtered_bounding_box_midpoint - camera_frame_midpoint[0]) < .05) 
            {
                robot_centered.set_value(true);
                RCLCPP_INFO(temp_node->get_logger(), "Looking at bounding box with value %f", filtered_bounding_box_midpoint);
                RCLCPP_INFO(temp_node->get_logger(), "Test Failed");
                return;
            }
            else
            {
                RCLCPP_INFO(temp_node->get_logger(), "Looking at bounding box with value %f", filtered_bounding_box_midpoint);
                RCLCPP_INFO(temp_node->get_logger(), "Test Passed");
                return;
            }
    });
    rclcpp::spin_until_future_complete(temp_node, future);
}

void custom_assert(bool condition) {
    if (!condition) {
        std::cerr << "Assertion failed: " << std::endl;
    }
}

std::vector<uint32_t> findMidpointTest()
{
    std::vector<uint32_t> cornerUL {0, 100};
    std::vector<uint32_t> cornerUR {100, 100};
    std::vector<uint32_t> cornerBL {0, 200};
    std::vector<uint32_t> cornerBR {100, 200};
    std::vector<std::vector<uint32_t>> bounding_box;
    bounding_box.push_back(cornerUL);
    bounding_box.push_back(cornerUR);
    bounding_box.push_back(cornerBL);
    bounding_box.push_back(cornerBR);

    std::vector<uint32_t> actual_midpoint = findMidpoint(bounding_box);
    std::vector<uint32_t> expected_midpoint {50, 720-150};

    custom_assert(expected_midpoint == actual_midpoint);
    cout << "Expect: ";
    printVector(expected_midpoint);
    cout << "Answer was: ";
    printVector(actual_midpoint);
    std::cout << "Find Midpoint Test Passed" << std::endl;
    return expected_midpoint;
}

std::unique_ptr<std::vector<std::vector<uint32_t>>> ZedToRosBoundingBoxTest()
{
    scion_types::msg::Keypoint2Di cornerUL;
    scion_types::msg::Keypoint2Di cornerUR;
    scion_types::msg::Keypoint2Di cornerBL;
    scion_types::msg::Keypoint2Di cornerBR;
    cornerUL.kp[0] = 0;
    cornerUL.kp[1] = 100;
    cornerUR.kp[0] = 100;
    cornerUR.kp[1] = 100;
    cornerBL.kp[0] = 0;
    cornerBL.kp[1] = 200;
    cornerBR.kp[0] = 100;
    cornerBR.kp[1] = 200;

    std::array<scion_types::msg::Keypoint2Di, 4> zed_bounding_box = {cornerUL, cornerUR, cornerBL, cornerBR};

    std::unique_ptr<std::vector<std::vector<uint32_t>>> ros_bounding_box = zed_to_ros_bounding_box(zed_bounding_box);

    custom_assert((*ros_bounding_box)[0][0] == 0);
    std::cout << (*ros_bounding_box)[0][0] << std::endl;
    custom_assert((*ros_bounding_box)[0][1] == 100);
    std::cout << (*ros_bounding_box)[0][1] << std::endl;
    custom_assert((*ros_bounding_box)[1][0] == 100);
    std::cout << (*ros_bounding_box)[1][0] << std::endl;
    custom_assert((*ros_bounding_box)[1][1] == 100);
    std::cout << (*ros_bounding_box)[1][1] << std::endl;
    custom_assert((*ros_bounding_box)[2][0] == 0);
    std::cout << (*ros_bounding_box)[2][0] << std::endl;
    custom_assert((*ros_bounding_box)[2][1] == 200);
    std::cout << (*ros_bounding_box)[2][1] << std::endl;
    custom_assert((*ros_bounding_box)[3][0] == 100);
    std::cout << (*ros_bounding_box)[3][0] << std::endl;
    custom_assert((*ros_bounding_box)[3][1] == 200);
    std::cout << (*ros_bounding_box)[3][1] << std::endl;

    std::cout << "Zed To Ros Bounding Box Test Passed" << std::endl;
    return ros_bounding_box;
}

unique_ptr<Filter> populateFilterBufferTest()
{
    scion_types::msg::Keypoint2Di cornerUL;
    scion_types::msg::Keypoint2Di cornerUR;
    scion_types::msg::Keypoint2Di cornerBL;
    scion_types::msg::Keypoint2Di cornerBR;
    cornerUL.kp[0] = 0;
    cornerUL.kp[1] = 100;
    cornerUR.kp[0] = 100;
    cornerUR.kp[1] = 100;
    cornerBL.kp[0] = 0;
    cornerBL.kp[1] = 200;
    cornerBR.kp[0] = 100;
    cornerBR.kp[1] = 200;
    std::array<scion_types::msg::Keypoint2Di, 4> zed_bounding_box = {cornerUL, cornerUR, cornerBL, cornerBR};

    Interface::node_t temp_node = rclcpp::Node::make_shared("zed_vision_publisher");
    Interface::vision_pub_t object_pub = temp_node->create_publisher<scion_types::msg::ZedObject>("zed_vision_data", 10);
    Interface::ros_timer_t object_timer = temp_node->create_wall_timer(200ms, [&object_pub, &zed_bounding_box](){
        scion_types::msg::ZedObject msg1 = scion_types::msg::ZedObject();
        msg1.label_id = 0;
        msg1.corners = zed_bounding_box;
        object_pub->publish(msg1);
    });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(temp_node);
    std::thread([&executor]() {
        executor.spin();
    }).detach();

    unique_ptr<Filter> moving_average_filter = populateFilterBuffer(0);
    for (float value : moving_average_filter->data_streams[0])
    {
        RCLCPP_INFO(temp_node->get_logger(), "%f", value);
        custom_assert(abs(value - 50) < .05);
    }
    RCLCPP_INFO(temp_node->get_logger(), "populateFilterBufferTest Passed");
}

void centerRobotTest()
{
    scion_types::msg::Keypoint2Di cornerUL;
    scion_types::msg::Keypoint2Di cornerUR;
    scion_types::msg::Keypoint2Di cornerBL;
    scion_types::msg::Keypoint2Di cornerBR;
    cornerUL.kp[0] = 0;
    cornerUL.kp[1] = 100;
    cornerUR.kp[0] = 100;
    cornerUR.kp[1] = 100;
    cornerBL.kp[0] = 0;
    cornerBL.kp[1] = 200;
    cornerBR.kp[0] = 100;
    cornerBR.kp[1] = 200;
    std::array<scion_types::msg::Keypoint2Di, 4> zed_bounding_box = {cornerUL, cornerUR, cornerBL, cornerBR};
    Interface::node_t temp_node = rclcpp::Node::make_shared("zed_vision_publisher");
    Interface::vision_pub_t object_pub = temp_node->create_publisher<scion_types::msg::ZedObject>("zed_vision_data", 10);
    Interface::ros_timer_t object_timer = temp_node->create_wall_timer(200ms, [&object_pub, &zed_bounding_box](){
        scion_types::msg::ZedObject msg1 = scion_types::msg::ZedObject();
        msg1.label_id = 0;
        msg1.corners = zed_bounding_box;
        object_pub->publish(msg1);
    });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(temp_node);
    std::thread([&executor]() {
        executor.spin();
    }).detach();
    centerRobot(0);
}

void centerRobotRightTest()
{
    scion_types::msg::Keypoint2Di cornerUL;
    scion_types::msg::Keypoint2Di cornerUR;
    scion_types::msg::Keypoint2Di cornerBL;
    scion_types::msg::Keypoint2Di cornerBR;
    cornerUL.kp[0] = 700;
    cornerUL.kp[1] = 100;
    cornerUR.kp[0] = 900;
    cornerUR.kp[1] = 100;
    cornerBL.kp[0] = 700;
    cornerBL.kp[1] = 200;
    cornerBR.kp[0] = 900;
    cornerBR.kp[1] = 200;
    std::array<scion_types::msg::Keypoint2Di, 4> zed_bounding_box = {cornerUL, cornerUR, cornerBL, cornerBR};
    Interface::node_t temp_node = rclcpp::Node::make_shared("zed_vision_publisher");
    Interface::vision_pub_t object_pub = temp_node->create_publisher<scion_types::msg::ZedObject>("zed_vision_data", 10);
    Interface::ros_timer_t object_timer = temp_node->create_wall_timer(200ms, [&object_pub, &zed_bounding_box](){
        scion_types::msg::ZedObject msg1 = scion_types::msg::ZedObject();
        msg1.label_id = 0;
        msg1.corners = zed_bounding_box;
        object_pub->publish(msg1);
    });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(temp_node);
    std::thread([&executor]() {
        executor.spin();
    }).detach();
    centerRobotRight(0);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::cout << "____________________________________________________________________________" << std::endl;
    findMidpointTest();
    std::cout << std::endl;
    ZedToRosBoundingBoxTest();
    std::cout << std::endl;
    populateFilterBufferTest();
    std::cout << std::endl;
    centerRobotTest();
    std::cout << std::endl;
    centerRobotRightTest();
    rclcpp::shutdown();
    return 0;
}