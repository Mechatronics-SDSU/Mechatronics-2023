#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "brain_node.hpp"
#include "pid_node.hpp"
#include "current_state_node.hpp"

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

TEST_F(ROBOT_TEST_SUITE, ActivatePids) {
    shared_ptr<Brain> brain_ptr = std::make_shared<Brain>();
    shared_ptr<CurrentStateNode> current_state_ptr = std::make_shared<CurrentStateNode>();
    shared_ptr<Controller> pid_ptr = std::make_shared<Controller>();

    
}
