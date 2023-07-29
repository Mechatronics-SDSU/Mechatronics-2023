#ifndef BRAIN_H
#define BRAIN_H

#include <vector>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <cmath>

#include "scion_types/action/pid.hpp"
#include "control_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "vector_operations.hpp"
#include "filter.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;

#define TO_THE_RIGHT            4.0f
#define TO_THE_LEFT             -4.0f
#define PIXEL_ERROR_THRESHOLD   100
#define SLEEP_TIME              50ms
#define SMOOTH_TURN_DEGREE      25.0f
#define SMOOTH_MOVE_DEGREE      1.0f
#define SUBMERGE_DISTANCE       1.5f
#define MID_X_PIXEL             640
#define MID_Y_PIXEL             360
#define NUM_CORNERS             4
#define BLIND_THRESHOLD         50
#define BUTTON_PRESS            4
#define EXTERNAL_MESSAGE        5

class Brain : public rclcpp::Node
{
    typedef bool (Brain::*condition_t)(string);
    typedef void (Brain::*action_t)(float);
    typedef void (Brain::*cleanup_t)();

    public:
        explicit Brain();

    // private:
        Interface::idea_pub_t                       idea_pub_;
        Interface::ros_timer_t                      decision_timer_;
        Interface::idea_vector_t                    idea_sequence_;
        Interface::object_sub_t                     object_sub_;
        Interface::int_sub_t                        commands_in_queue_sub_;
        Interface::ros_sendframe_client_t           can_client_;
        Interface::ros_trigger_service_t            pid_ready_service_;
        Interface::ros_trigger_service_t            vision_ready_service_;
        float                                       lastFilteredMidpoint_;
        float                                       lastUnFilteredMidpoint_;
        std::string                                 mode_param_;
        bool                                        object_seen_ = false;
        ////////////////////////////////////////////////////////////////////////////////
        //                               INIT MISSION                                 //
        ////////////////////////////////////////////////////////////////////////////////

        void ready(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        ////////////////////////////////////////////////////////////////////////////////
        //                               ASYNC FUNCTIONS                              //
        ////////////////////////////////////////////////////////////////////////////////

        void doUntil(action_t action, condition_t condition, cleanup_t cleanup, bool& condition_global, string condition_param, float action_param);
        void waitForReady();
        bool objectSeen(string object);
        float getDistanceFromCamera(string object);
        unique_ptr<Filter> populateFilterBuffer(int object_identifier);
        void centerRobot(int object_identifier);
        std::unique_ptr<vector<vector<uint32_t>>> zed_to_ros_bounding_box(std::array<scion_types::msg::Keypoint2Di, 4>& zed_bounding_box);
        vector<uint32_t> findMidpoint(vector<vector<uint32_t>>& bounding_box);
        bool areEqual(vector<uint32_t> point_a, vector<uint32_t> point_b);
        void adjustToCenter(float bounding_box_midpoint, float camera_frame_midpoint);
        void adjustToCenter(vector<uint32_t> bounding_box_midpoint, vector<uint32_t> camera_frame_midpoint);
        void waitForCommandQueueEmpty();
        bool isCommandQueueEmpty();

        ////////////////////////////////////////////////////////////////////////////////
        //                               MOVEMENT IDEAS                               //
        ////////////////////////////////////////////////////////////////////////////////

        void stop();
        void turn(float degree);
        void pitch(float degree);
        void roll(float degree);
        void moveForward(float degree);
        void translate(float degree);
        void levitate(float degree);
        void keepTurning(float power);
        void keepMoving(float power);
        ////////////////////////////////////////////////////////////////////////////////
        //                                  MISSION                                   //
        ////////////////////////////////////////////////////////////////////////////////

        void performMission();

        ////////////////////////////////////////////////////////////////////////////////
        //                                  CALLBACK                                  //
        ////////////////////////////////////////////////////////////////////////////////
}; 















































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

#endif // BRAIN_H