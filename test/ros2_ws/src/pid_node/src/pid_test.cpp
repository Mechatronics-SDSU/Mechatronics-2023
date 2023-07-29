#include "pid_node.hpp"
#include "gtest/gtest.h"

class ROBOT_TEST_SUITE : public ::testing::Test {
public:
    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
    }

    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }
};

TEST_F(ROBOT_TEST_SUITE, slew_rate) {
    shared_ptr<Controller> pid_ptr = std::make_shared<Controller>();

    deque<vector<int>> slew_buffer;
    int i = 0;
    while (i < 11)
    {
        vector<int> motor_values {i,i,i,i,i,i};
        slew_buffer.push_back(motor_values);
        i++;
    } 
    float total_slew = pid_ptr->calculateTotalSlew(slew_buffer);
    EXPECT_EQ(total_slew, 60);
}

// int main(int argc, char ** argv)
// {
//   ::testing::InitGoogleTest(&argc, argv);
//   return RUN_ALL_TESTS();
// }