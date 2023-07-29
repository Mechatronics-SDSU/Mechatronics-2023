#include "brain_node.hpp"
#include "pid_node.hpp"
#include "gtest/gtest.h"
#include "gmock/gmock.h"

using ::testing::AtLeast;                         // #1

class MockBrain : public Brain
{
public:
    MOCK_METHOD(void, ready, (const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response), (override));
    MOCK_METHOD(void, centerRobot, (int object_identifier), (override));
};

class ROBOT_TEST_SUITE : public ::testing::Test {
public:
    friend class Brain;

    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
    }

    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }
};

TEST_F(ROBOT_TEST_SUITE, Midpoint) {
    shared_ptr<Brain> brain_ptr = std::make_shared<Brain>();

    std::vector<uint32_t> cornerUL {0, 100};
    std::vector<uint32_t> cornerUR {100, 100};
    std::vector<uint32_t> cornerBL {0, 200};
    std::vector<uint32_t> cornerBR {100, 200};
    std::vector<std::vector<uint32_t>> bounding_box;
    bounding_box.push_back(cornerUL);
    bounding_box.push_back(cornerUR);
    bounding_box.push_back(cornerBL);
    bounding_box.push_back(cornerBR);

    std::vector<uint32_t> actual_midpoint = brain_ptr->findMidpoint(bounding_box);
    std::vector<uint32_t> expected_midpoint {50, 720-150};

    EXPECT_EQ(expected_midpoint, actual_midpoint);
}

TEST_F(ROBOT_TEST_SUITE, zed_to_ros) {
    shared_ptr<Brain> brain_ptr = std::make_shared<Brain>();

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
    std::unique_ptr<std::vector<std::vector<uint32_t>>> ros_bounding_box = brain_ptr->zed_to_ros_bounding_box(zed_bounding_box);

    EXPECT_EQ((*ros_bounding_box)[0][0], 0);
    EXPECT_EQ((*ros_bounding_box)[0][1], 100);
    EXPECT_EQ((*ros_bounding_box)[1][0], 100);
    EXPECT_EQ((*ros_bounding_box)[1][1], 100);
    EXPECT_EQ((*ros_bounding_box)[2][0], 0);
    EXPECT_EQ((*ros_bounding_box)[2][1], 200);
    EXPECT_EQ((*ros_bounding_box)[3][0], 100);
    EXPECT_EQ((*ros_bounding_box)[3][1], 200);
}

// TEST_F(ROBOT_TEST_SUITE, testReady) {
//     std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::shared_ptr<std_srvs::srv::Trigger::Request>();
//     std::shared_ptr<std_srvs::srv::Trigger::Response> response = std::shared_ptr<std_srvs::srv::Trigger::Response>();
//     shared_ptr<MockBrain> mock_brain_ptr = make_shared<MockBrain>();
//     EXPECT_CALL(*mock_brain_ptr, ready(request, response));
//     EXPECT_CALL(*mock_brain_ptr, centerRobot(0));
//     shared_ptr<Controller> pid_ptr = make_shared<Controller>();
//     pid_ptr->pidReady();
// }

// TEST_F(ROBOT_TEST_SUITE, fill_buffer) {
//     shared_ptr<Brain> brain_ptr = std::make_shared<Brain>();

//     scion_types::msg::Keypoint2Di cornerUL;
//     scion_types::msg::Keypoint2Di cornerUR;
//     scion_types::msg::Keypoint2Di cornerBL;
//     scion_types::msg::Keypoint2Di cornerBR;
//     cornerUL.kp[0] = 0;
//     cornerUL.kp[1] = 100;
//     cornerUR.kp[0] = 100;
//     cornerUR.kp[1] = 100;
//     cornerBL.kp[0] = 0;
//     cornerBL.kp[1] = 200;
//     cornerBR.kp[0] = 100;
//     cornerBR.kp[1] = 200;
//     std::array<scion_types::msg::Keypoint2Di, 4> zed_bounding_box = {cornerUL, cornerUR, cornerBL, cornerBR};
//     Interface::node_t temp_node = rclcpp::Node::make_shared("zed_vision_publisher");
//     Interface::vision_pub_t object_pub = temp_node->create_publisher<scion_types::msg::ZedObject>("zed_vision_data", 10);
//     Interface::ros_timer_t object_timer = temp_node->create_wall_timer(500ms, [&object_pub, &zed_bounding_box](){
//         scion_types::msg::ZedObject msg1 = scion_types::msg::ZedObject();
//         msg1.label_id = 0;
//         msg1.corners = zed_bounding_box;
//         object_pub->publish(msg1);
//     });
// //     rclcpp::executors::SingleThreadedExecutor executor;
// //     executor.add_node(temp_node);
// //     std::thread([&executor]() {
// //         executor.spin();
// //     }).detach();

//     std::shared_future<unique_ptr<Filter>> filled_buffer = std::async(std::launch::async, &Brain::populateFilterBuffer, brain_ptr, 0);
//     rclcpp::spin_until_future_complete(temp_node, filled_buffer);

// //     unique_ptr<Filter> moving_average_filter = brain_ptr->populateFilterBuffer(0);
//     for (float value : filled_buffer.get()->data_streams[0])
//     {
//         EXPECT_LT(abs(value - 50), .05);
//     }
// }

void async_fill_buffer_test() {
    rclcpp::init(0, nullptr);
    shared_ptr<Brain> brain_ptr = std::make_shared<Brain>();

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
    Interface::ros_timer_t object_timer = temp_node->create_wall_timer(50ms, [&object_pub, &zed_bounding_box](){
        scion_types::msg::ZedObject msg1 = scion_types::msg::ZedObject();
        msg1.label_id = 0;
        msg1.corners = zed_bounding_box;
        object_pub->publish(msg1);
    });
//     rclcpp::executors::SingleThreadedExecutor executor;
//     executor.add_node(temp_node);
//     std::thread([&executor]() {
//         executor.spin();
//     }).detach();

    std::shared_future<unique_ptr<Filter>> filled_buffer = std::async(std::launch::async, &Brain::populateFilterBuffer, brain_ptr, 0);
    rclcpp::spin_until_future_complete(temp_node, filled_buffer);

//     unique_ptr<Filter> moving_average_filter = brain_ptr->populateFilterBuffer(0);
    for (float value : filled_buffer.get()->data_streams[0])
    {
        EXPECT_LT(abs(value - 50), .05);
    }
    RCLCPP_INFO(brain_ptr->get_logger(), "Buffer Was Filled");
    rclcpp::shutdown();
}

void async_test() {
    rclcpp::init(0, nullptr);
    shared_ptr<Brain> brain_ptr = std::make_shared<Brain>();
    rclcpp::shutdown();
}


int main(int argc, char ** argv)
{
    async_fill_buffer_test();
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}