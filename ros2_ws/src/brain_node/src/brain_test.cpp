#include "brain_node.hpp"
#include "gtest/gtest.h"

TEST(TestValues, IntegerValue) {
  rclcpp::init(0, nullptr);
  shared_ptr<Brain> brain_ptr = std::make_shared<Brain>();
  rclcpp::shutdown();

  EXPECT_EQ(brain_ptr->int_, 3);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}