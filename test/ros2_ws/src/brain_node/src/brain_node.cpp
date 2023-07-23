#include <vector>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <cmath>
#include <cstdlib>

#include "scion_types/msg/keypoint2_di.hpp"
#include "control_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vector_operations.hpp"


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

void custom_assert(bool condition) {
    if (!condition) {
        std::cerr << "Assertion failed: " << std::endl;
    }
}

void findMidpointTest()
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
}

void zed_to_ros_bounding_box_test()
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
}

int main()
{
    findMidpointTest();
    std::cout << std::endl;
    zed_to_ros_bounding_box_test();
    return 0;
}