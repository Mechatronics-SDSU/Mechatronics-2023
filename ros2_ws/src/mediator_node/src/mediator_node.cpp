/* 
 * @author Zix
 * Command Queue Node, translates ideas and puts them into a queue to give a desired state to PIDs
 */

#include "mediator_node.hpp"

using PIDAction = scion_types::action::PID;
using GoalHandlePIDAction = rclcpp_action::ClientGoalHandle<PIDAction>;
using namespace std::placeholders;
using namespace std::literals::chrono_literals;

Mediator::Mediator() : Node("Mediator")
{
    this->pid_command_client_ = rclcpp_action::create_client<PIDAction>
    (
      this,
      "PIDAction"
    );

    reset_relative_state_client_ = this->create_client<std_srvs::srv::Trigger>("reset_relative_state");
    reset_relative_position_client_ = this->create_client<std_srvs::srv::Trigger>("reset_relative_position");
    use_position_client_ = this->create_client<std_srvs::srv::SetBool>("use_position");
    stop_robot_client_ = this->create_client<std_srvs::srv::Trigger>("stop_robot");
    stabilize_robot_client_ = this->create_client<std_srvs::srv::SetBool>("stabilize_robot");
    commands_in_queue_count_pub_ = this->create_publisher<std_msgs::msg::Int32>("is_command_queue_empty", 10);
    commands_count_timer_ = this->create_wall_timer(50ms, std::bind(&Mediator::commands_count_timer_callback, this));

    current_state_sub_ = this->create_subscription<scion_types::msg::State>
    (
      "relative_current_state_data", 
      10, 
      std::bind(&Mediator::current_state_callback, this, _1)
    );

    idea_sub_ = this->create_subscription<scion_types::msg::Idea>
    (
      "brain_idea_data",
        10,
        std::bind(&Mediator::translateIdea, this, _1)
    );

    current_command_ = nullptr;

    this->next_command_timer_ = this->create_wall_timer
    (
      std::chrono::milliseconds(100), 
      std::bind(&Mediator::nextCommand, this)
    );
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
                                        // COMMAND HANDLING // 
/////////////////////////////////////////////////////////////////////////////////////////////////////

void Mediator::nextCommandPrep(Interface::state_transform_func command)
{
    this->resetPosition();
}

Interface::desired_state_t Mediator::adjustDesiredState(Interface::desired_state_t& desired_state, Interface::state_transform_func command_function)
{
    if (command_function == &Movements::move)
    {
        float xError = desired_state[3];
        desired_state[3] = xError * cos(this->current_state_[0] * (M_PI/180));
        desired_state[4] = xError * sin(this->current_state_[0] * (M_PI/180));
    }
    if (command_function == &Movements::translate)
    {
        float yError = desired_state[4];
        desired_state[3] = yError * sin(this->current_state_[0] * (M_PI/180));
        desired_state[4] = yError * cos(this->current_state_[0] * (M_PI/180));
    }

    return desired_state;
}

void Mediator::commandCleanup()
{
  /* Success State */
  sleep(.01); 
  current_command_ = nullptr;
}

void Mediator::nextCommand()
{
    using namespace Interface;
    if (this->command_queue_.size() > 0 && current_command_ == nullptr) // && controlInit == true
    {
        this->current_command_ = &command_queue_[0];
        this->command_queue_.pop_front();
        state_transform_func command_function = current_command_->function.transform;
        this->nextCommandPrep(command_function);
        desired_state_t desired = (*command_function)(current_command_->params.degree);
        desired = adjustDesiredState(desired, command_function);
        current_state_t current = getCurrentState();
        desired += current;
        this->send_goal(desired);
    }
}

Interface::current_state_t Mediator::getCurrentState()
{
    std::promise<Interface::current_state_t> current_state;
    std::shared_future<Interface::current_state_t> future  = current_state.get_future();
    Interface::node_t temp_node = rclcpp::Node::make_shared("temp_state_subscriber");;
    Interface::state_sub_t current_state_sub = temp_node->create_subscription<scion_types::msg::State>
    ("relative_current_state_data", 10, [&temp_node, &current_state](const scion_types::msg::State::SharedPtr msg) {
            current_state.set_value(msg->state);
    });
    rclcpp::spin_until_future_complete(temp_node, future);
    return future.get();
}

void Mediator::addToQueue(Interface::command_vector_t& command_vector)
{
    for (Interface::Command command : command_vector)
    {
      this->command_queue_.push_back(command);
    }
}

void Mediator::translateIdea(scion_types::msg::Idea::SharedPtr idea)
{
  using namespace Interface;
  Interface::command_vector_t command_vector;
  switch (idea->code)
  {
      case Idea::STOP:
          this->cancel_goal();
        break;
      case Idea::TURN:
        command_vector = Translator::turn(idea->parameters[0]);
        addToQueue(command_vector);
        break;
      case Idea::PITCH:
        command_vector = Translator::pitch(idea->parameters[0]);
        addToQueue(command_vector);
        break;
      case Idea::ROLL:
        command_vector = Translator::roll(idea->parameters[0]);
        addToQueue(command_vector);
        break;
      case Idea::MOVE:
        command_vector = Translator::move(idea->parameters[0]);
        addToQueue(command_vector);
        break;
      case Idea::TRANSLATE:
        command_vector = Translator::translate(idea->parameters[0]);
        addToQueue(command_vector);
        break;
      case Idea::LEVITATE:
        command_vector = Translator::levitate(idea->parameters[0]);
        addToQueue(command_vector);
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

/////////////////////////////////////////////////////////////////////////////////////////////////////
                                        // ACTION SERVER // 
/////////////////////////////////////////////////////////////////////////////////////////////////////


void Mediator::send_goal(Interface::desired_state_t& desired)
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

  void Mediator::cancel_goal()
{
    using namespace std::placeholders;
    auto cancel_goal_options = rclcpp_action::Client<PIDAction>::CancelRequest();
    this->pid_command_client_->async_cancel_all_goals();
    this->stopRobot();
    this->clearQueue(this->command_queue_);
}

void Mediator::clearQueue(std::deque<Interface::Command> &q)
{
    std::deque<Interface::Command> empty;
    std::swap( q, empty );
}

void Mediator::goal_response_callback
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

void Mediator::feedback_callback
(
  GoalHandlePIDAction::SharedPtr, const std::shared_ptr<const PIDAction::Feedback> feedback
)
{

}

void Mediator::result_callback
(
  const GoalHandlePIDAction::WrappedResult & result
)
{
    switch (result.code) 
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        this->commandCleanup();
        break;
      case rclcpp_action::ResultCode::ABORTED:
        this->commandCleanup();
        RCLCPP_INFO(this->get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(this->get_logger(), "Goal was canceled");
        this->commandCleanup();
        break;
      default:
        RCLCPP_INFO(this->get_logger(), "Unknown result code");
        break;
    }   
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
                                        // MEDIATION SERVICES // 
/////////////////////////////////////////////////////////////////////////////////////////////////////

void Mediator::stopRobot()
{
    auto stop_robot_request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto stop_robot_future = this->stop_robot_client_->async_send_request(stop_robot_request);
}

void Mediator::resetState()
{     
    auto reset_state_request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto reset_state_future = this->reset_relative_state_client_->async_send_request(reset_state_request);
}

void Mediator::resetPosition()
{     
    auto reset_position_request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto reset_position_future = this->reset_relative_position_client_->async_send_request(reset_position_request);
}

void Mediator::usePositionRequest(bool request)
{
    auto use_position_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    use_position_request->data = request;
    auto use_position_result = this->use_position_client_->async_send_request(use_position_request);
}

void Mediator::stabilizeRobotRequest(bool request)
{
    auto stabilize_robot_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    stabilize_robot_request->data = request;
    auto stabilize_robot_result = this->stabilize_robot_client_->async_send_request(stabilize_robot_request);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
                                        // CALLBACK FUNCTIONS // 
/////////////////////////////////////////////////////////////////////////////////////////////////////

void Mediator::commands_count_timer_callback()
{
    auto message = std_msgs::msg::Int32();
    message.data = (this->command_queue_.size() == 0 && this->current_command_ == nullptr);
    this->commands_in_queue_count_pub_->publish(message);
}

void Mediator::current_state_callback(const scion_types::msg::State::SharedPtr msg)
{
    this->current_state_= msg->state; 
}