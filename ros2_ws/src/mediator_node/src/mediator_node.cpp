/* @author Zix
 * I'll explain this later there's a lot going on haha.
 */


#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <iostream>
#include <unistd.h>
#include <deque>

#include "scion_types/action/pid.hpp"
#include "scion_types/msg/state.hpp"
#include "scion_types/msg/idea.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "control_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using PIDAction = scion_types::action::PID;
using GoalHandlePIDAction = rclcpp_action::ClientGoalHandle<PIDAction>;
using namespace std::placeholders;

class Mediator : public rclcpp::Node
{
public:
  explicit Mediator()
  : Node("Mediator")
  {
    this->pid_command_client_ = rclcpp_action::create_client<PIDAction>
    (
      this,
      "PIDAction"
    );

    reset_relative_state_client_ = this->create_client<std_srvs::srv::Trigger>("reset_relative_state");

    idea_sub_ = this->create_subscription<scion_types::msg::Idea>
    (
      "brain_idea_data",
       10,
       std::bind(&Mediator::translateIdea, this, _1)
    );

    // this->populateQueue();
    current_command_ = nullptr;

    this->next_command_timer_ = this->create_wall_timer
    (
      std::chrono::milliseconds(100), 
      std::bind(&Mediator::nextCommand, this)
    );
  }

private:
  rclcpp_action::Client<PIDAction>::SharedPtr pid_command_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_relative_state_client_;
  rclcpp::Subscription<scion_types::msg::Idea>::SharedPtr idea_sub_;
  rclcpp::TimerBase::SharedPtr next_command_timer_;
  Interface::command_queue_t command_queue_;
  Interface::Command* current_command_; 


  void addToQueue(Interface::command_vector_t& command_vector)
  {
    for (Interface::Command command : command_vector)
    {
      this->command_queue_.push_back(command);
    }
  }

  void translateIdea(scion_types::msg::Idea::SharedPtr idea)
    {
      using namespace Interface;
      Interface::command_vector_t command_vector;
      switch (idea->code)
      {
        case Idea::STOP:
          Translator::stop();
          break;
        case Idea::GO:
          Translator::go(idea->parameters[0]);
          break;
        case Idea::SPIN:
          Translator::spin(idea->parameters[0]);
          break;
        case Idea::MOVE:
          Translator::move(idea->parameters[0]);
          break;
        case Idea::TURN:
          Translator::turn(idea->parameters[0]);
          break;
        case Idea::RELATIVE_POINT:
          command_vector = Translator::relativePoint(idea->parameters[0], idea->parameters[1]);
          addToQueue(command_vector);
          break;
        case Idea::ABSOLUTE_POINT:
          Translator::absolutePoint(idea->parameters[0], idea->parameters[1]);
          break;
        case Idea::PURE_RELATIVE_POINT:
          Translator::pureRelativePoint(idea->parameters[0], idea->parameters[1]);
          break;
        case Idea::PURE_ABSOLUTE_POINT:
          Translator::pureAbsolutePoint(idea->parameters[0], idea->parameters[1]);
          break;
      }
    }

  void send_goal(Interface::desired_state_t& desired)
  {
    using namespace std::placeholders;
    if (!this->pid_command_client_->wait_for_action_server()) 
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }
    auto goal_msg = PIDAction::Goal();
    goal_msg.desired_state = desired;
    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<PIDAction>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&Mediator::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&Mediator::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&Mediator::result_callback, this, _1);
    this->pid_command_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void cancel_goal()
  {
    using namespace std::placeholders;
    auto cancel_goal_options = rclcpp_action::Client<PIDAction>::CancelRequest();
    this->pid_command_client_->async_cancel_all_goals();
  }

  void goal_response_callback
  (
    std::shared_future<GoalHandlePIDAction::SharedPtr> future
  )
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback
  (
    GoalHandlePIDAction::SharedPtr, const std::shared_ptr<const PIDAction::Feedback> feedback
  )
  {
    // std::stringstream ss;
    // ss << "Current State: ";
    // for (float element : feedback->ongoing_state) {
    //   ss << element << " ";
    // }
    // RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback
  (
    const GoalHandlePIDAction::WrappedResult & result
  )
  {
    switch (result.code) 
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    /* SUCCESS STATE */
    std::stringstream ss;
    ss << "State Accomplished; Setting Current Command to Null\n";
    sleep(.3); // Sleep after reaching desired state for a split second before taking out current command
    auto reset_state_request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto reset_state_result = this->reset_relative_state_client_->async_send_request(reset_state_request);
    current_command_ = nullptr;
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void nextCommand()
  {
    using namespace Interface;
    if (this->command_queue_.size() > 0 && current_command_ == nullptr) // && controlInit == true
    {
      this->current_command_ = &command_queue_[0];
      this->command_queue_.pop_front();
      
      state_transform_func func = current_command_->function.transform;
      desired_state_t desired = (*func)(current_command_->params.degree);
      this->send_goal(desired);
    }
  }

//   void populateQueue()
//   {
//     using namespace Interface;
//     Command command1;
//     command1.function.movement = &Movements::count;
//     Command command2;
//     command2.function.movement = &Movements::count;
//     Command command3;
//     command3.function.movement = &Movements::count;

//     this->command_queue_.push_back(command1);
//     this->command_queue_.push_back(command2);
//     this->command_queue_.push_back(command3);
//   }

};  // class Mediator

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mediator>());
  rclcpp::shutdown();
  return 0;
}