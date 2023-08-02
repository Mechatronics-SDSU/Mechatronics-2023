/* 
 * @author Conner Sommerfield - Zix on Discord
 * The infamous PID Controller Node
 * Subscribes to all sensor data needed for control system and throws it into a PID object
 * to output ctrl and thrust values.
 * Refer to classes/pid_controller for more info
 * 
 * Every time the subscription callback is triggered, the current state will be updated
 * Current state is stored in member variables of the Node
 * There is a timer that updates every x amount of ms based on what user inputs
 * This calls the PID update function which will use that sensor data to get ctrl/thrust values
 * Then it makes a CAN Request using that array of thrust_vals to send to motors
 * CAN Request is of ID #010
 */

#include "pid_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;

#define SLEEP_TIME 300
#define UPDATE_PERIOD 100ms
#define UPDATE_PERIOD_RAW 10
#define PRINT_PERIOD 500ms
#define PID_ERROR_THRESHOLD 0.01f
#define MOTOR_ID 0x010
#define ENABLE_BYTE 0x00A

/////////////////////////////////////////////////////////////////////////////////////////////////////
                                    // MEMBER VARIABLE DECLARATIONS // 
/////////////////////////////////////////////////////////////////////////////////////////////////////

using PIDAction = scion_types::action::PID;
using GoalHandlePIDAction = rclcpp_action::ServerGoalHandle<PIDAction>;

/** 
 * Controller Node consists of: 
 *      - timer to update ctrl_vals
 *      - subscription to desired state
 *      - subscription to ms5837 depth data
 *      - subscription to ZED position data
 *      - subscription to AHRS orientation data
 *      - Creates CAN Mailbox and ScionPIDController objects 
**/

Controller::Controller(): Node("pid_controller")
{

    this->declare_parameter("thrust_mapper", "percy");

    string thrust_mapper = this->get_parameter("thrust_mapper").as_string();

    if (thrust_mapper == "percy")
    {
        this->thrust_mapper_ = Interface::percy_thrust_mapper;
    } 
    else if (thrust_mapper == "junebug")
    {
        this->thrust_mapper_ = Interface::junebug_thrust_mapper;
        this->motor_count_ = 2;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "GIVE VALID VALUE FOR THRUST MAPPER IN LAUNCH FILE");
        exit(EXIT_FAILURE);
    }

    update_timer_ = this->create_wall_timer(UPDATE_PERIOD, std::bind(&Controller::update_timer_callback, this));
    
    print_timer_ = this->create_wall_timer(PRINT_PERIOD, std::bind(&Controller::print_timer_callback, this));

    current_state_sub_ = this->create_subscription<scion_types::msg::State>
    ("relative_current_state_data", 10, std::bind(&Controller::current_state_callback, this, _1));

    axis_tuning_map_ = std::map<int, string> 
    ({
        {0, "yaw"  }, 
        {1, "pitch"},
        {2, "roll" },
        {3, "x_pos"},
        {4, "y_pos"},
        {5, "z_pos"},
    });

    kp_tuning_sub_ = this->create_subscription<scion_types::msg::PidTuning>("kp_dial_data", 10, std::bind(&Controller::kp_tuning_callback, this, _1));
    ki_tuning_sub_ = this->create_subscription<scion_types::msg::PidTuning>("ki_dial_data", 10, std::bind(&Controller::ki_tuning_callback, this, _1));
    kd_tuning_sub_ = this->create_subscription<scion_types::msg::PidTuning>("kd_dial_data", 10, std::bind(&Controller::kd_tuning_callback, this, _1));

    // desired_state_sub_ = this->create_subscription<scion_types::msg::State>
    // ("desired_state_data", 10, std::bind(&Controller::desired_state_callback, this, _1));

    this->pid_command_server_ = rclcpp_action::create_server<PIDAction>
    (
        this,
        "PIDAction",
        std::bind(&Controller::handle_goal, this, _1, _2),
        std::bind(&Controller::handle_cancel, this, _1),
        std::bind(&Controller::handle_accepted, this, _1)
    );

    can_client_ = this->create_client<scion_types::srv::SendFrame>("send_can_raw");
    reset_relative_state_client_ = this->create_client<std_srvs::srv::Trigger>("reset_relative_state");
    reset_relative_position_client_ = this->create_client<std_srvs::srv::Trigger>("reset_relative_position");
    pid_ready_client_ = this->create_client<std_srvs::srv::Trigger>("pid_ready");
    stop_robot_service_ = this->create_service<std_srvs::srv::Trigger>("stop_robot", std::bind(&Controller::stopRobot, this, _1, _2));
    use_position_service_ = this->create_service<std_srvs::srv::SetBool>("use_position", std::bind(&Controller::usePosition, this, _1, _2));
    stabilize_robot_service_ = this->create_service<std_srvs::srv::SetBool>("stabilize_robot", std::bind(&Controller::stabilizeRobot, this, _1, _2));

    submarine_state_sub_ = this->create_subscription<scion_types::msg::SubState>("submarine_state", 10, [this](const scion_types::msg::SubState::SharedPtr msg)
      {
          if (msg->host_mode == 0) 
          {
              exit(EXIT_SUCCESS);
          }
      });

    controller_ = Scion_Position_PID_Controller(pid_params_object_.get_pid_params());
    controller_.getStatus();

    auto initFunction = std::bind(&Controller::initDesiredState, this);
    std::thread(initFunction).detach();
}

    /////////////////////////////////////////////////////////////////////////////////////////////////////
                                // WAIT FOR VALID DATA TO INITIALIZE PIDs // 
    /////////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::initDesiredState()
{
    auto desiredValid = std::bind(&Controller::desiredStateValid, this);
    std::future<bool> promise = std::async(desiredValid);
    std::cout << "Waiting for Valid Sensor Info. \n";
    bool valid = promise.get();
    std::cout << "Got Valid Sensor Info. \n";
}

bool Controller::desiredStateValid()
{
    while (!this->current_state_valid_)
    {
        this->sendNothingAndWait();
        std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME));
    }
    this->resetState();
    // std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME));
    this->desired_state_ = vector<float>{0,0,0,0,0,0};
    while (!this->desired_state_valid_) {this->desired_state_valid_ = true;}
    while (!this->current_state_valid_) {this->current_state_valid_ = true;}
    canClient::sendFrame(ENABLE_BYTE, 0, nothing_.data(), can_client_);
    canClient::setBotInSafeMode(can_client_);
    this->pidReady();
    return true;
}

void Controller::printCurrentAndDesiredStates()
{
    std::cout << "DESIRED STATE: ";
    printVector(this->desired_state_);
    std::cout << "CURRENT STATE: ";
    printVector(this->current_state_);
}

void Controller::sendNothingAndWait()
{
    canClient::sendFrame(0x010, 8, nothing_.data(), can_client_); // Keep from exiting safe mode by sending 0 command
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
                                    // UPDATES OF STATE FOR PIDs //
/////////////////////////////////////////////////////////////////////////////////////////////////////


/* Different timer for printing and sensor updates */
void Controller::print_timer_callback()
{
    cout << "\n_________________________\n";
    this->printCurrentAndDesiredStates();
    cout << "___________________________\n\n";
    this->controller_.getStatus(); 
    stabilize_robot_ ? cout << "Stabilizing Robot" << " | " : cout << "Not Stabilizing Robot" << " | ";
    current_state_valid_ && desired_state_valid_ ? cout << "ROBOT IS UPDATING" << " | " : cout << "WAITING FOR GOOD SENSOR INFO" << " | ";
    use_position_ ? cout << "USING POSITION" << " | " : cout << "IGNORING POSITION" << endl;
}

/* Essential callback set to update PID state at certain interval */
void Controller::update_timer_callback()
{
    update_current_state();
}

void Controller::update_current_state() // ** Important Function **
{    
/* 
    * STEP 1: Update the PID Controller (meaning call the ScionPIDController object's
    * update function which generates ctrl_vals and show its status on the screen 
    */
    // sendNothingAndWait();
    if (stabilize_robot_ && current_state_valid_ && desired_state_valid_)
    {
        vector<float> thrusts(motor_count_, 0);
        thrusts = this->getThrusts(this->current_state_, this->desired_state_);
        make_CAN_request(thrusts);
    }
}

vector<float> Controller::getErrors(vector<float>& current_state, vector<float>& desired_state) 
{
    return desired_state - current_state;
}    

vector<float> Controller::getThrusts(vector<float>& current_state, vector<float>& desired_state)
{
    vector<float> errors                   {0.0F,0.0F,0.0F,0.0F,0.0F,0.0F}; 
    vector<float> adjustedErrors           {0.0F,0.0F,0.0F,0.0F,0.0F,0.0F};
    vector<float> ctrl_vals                {0.0F,0.0F,0.0F,0.0F,0.0F,0.0F}; 

    errors = getErrors(current_state,  desired_state);
    adjustedErrors = adjustErrors(errors);
    ctrl_vals = this->controller_.update(adjustedErrors, (float)UPDATE_PERIOD_RAW / 1000);
    return ctrlValsToThrusts(ctrl_vals);
}    

vector<float> Controller::ctrlValsToThrusts(vector<float>& ctrl_vals)
{
    return this->thrust_mapper_ * ctrl_vals;
}

float Controller::angleBetweenHereAndPoint(float x, float y)
{
    if (areEqual(y, 0, .01)) {return 90;}
    float point_angle_radians = atan(x / y);
    float point_angle_degrees = point_angle_radians * (180/PI);
    if (y < 0) {point_angle_degrees += 180;}
    return point_angle_degrees;
}

float Controller::distanceBetweenHereAndPoint(float x, float y)
{
    return sqrt(pow(x,2) + pow(y,2));
}

vector<float> Controller::adjustErrors(vector<float>& errors)
{
    float yaw = current_state_[0];

    float x = errors[3];
    float y = errors[4];
    float z = errors[5];

    float absoluteAngle = angleBetweenHereAndPoint(y ,x);
    float absoluteDistance = distanceBetweenHereAndPoint(y, x);

    float yawAdjustedX = absoluteDistance * cos((absoluteAngle - yaw) * M_PI/180);
    float yawAdjustedY = absoluteDistance * sin((absoluteAngle - yaw) * M_PI/180);

    float adjustedX = yawAdjustedX;
    float adjustedY = yawAdjustedY;
    float adjustedZ = z;
            
    vector<float> adjustedErrors = vector<float>
    {
        errors[0],
        errors[1],
        errors[2],
        adjustedX,
        adjustedY,
        adjustedZ
    };
    return adjustedErrors;
}

vector<float> Controller::update_PID(Interface::current_state_t& current_state, Interface::desired_state_t& desired_state)
{
    using namespace Interface;
    if (!this->current_state_valid_ || !this->desired_state_valid_) 
    {
        return vector<float>(this->motor_count_, 0);
    }

    if (this->use_position_)
    {
        return this->getThrusts(current_state, desired_state); 
    }
    else
    {
        current_state_t current_state_no_position = current_state_t{current_state[0], current_state[1], current_state[2], 0, 0, 0};
        desired_state_t desired_state_no_position = desired_state_t{desired_state[0], desired_state[1], desired_state[2], 0, 0, 0};
        return this->getThrusts(current_state_no_position, desired_state_no_position);    
    }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////
                                // CAN REQUESTS FOR MOTOR CONTROL // 
/////////////////////////////////////////////////////////////////////////////////////////////////////

vector<int> Controller::make_CAN_request(vector<float>& thrusts)
{
    /* Thrusts come out of PID as a float between -1 and 1; motors need int value from -100 to 100 */
    std::vector<int> convertedThrusts;
    for (float thrust : thrusts)
    {
        convertedThrusts.push_back(((int)(thrust * 100)));
    }

    /* 
        * We have integer values that are 32 bits (4 bytes) but need values of one byte to send to motor
        * We can extract using an and mask and get last 8 bits which in hex is 0xFF. Char size is one byte
        * which is why we use an array of chars
        */          

    std::vector<unsigned char> byteThrusts;
    for (int thrust : convertedThrusts)
    {
        byteThrusts.push_back((thrust & 0xFF));
    }
    /* See exactly our 8 thrust values sent to motors */
    cout << "THRUSTS: ";
    printVector(convertedThrusts);

////////////////////////////////////////// BUILD REQUEST //////////////////////////////////////////
    /* 
        * Our frame will send CAN request 
        * one byte for each value -100 to 100 
        */

    canClient::sendFrame(MOTOR_ID, motor_count_, byteThrusts.data(), this->can_client_);
    return convertedThrusts;
}

////////////////////////////////////////// END REQUEST //////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////
                                        // ACTION SERVER // 
/////////////////////////////////////////////////////////////////////////////////////////////////////


rclcpp_action::GoalResponse Controller::handle_goal
(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const PIDAction::Goal> goal
)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->desired_state);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Controller::handle_cancel
(
    const std::shared_ptr<GoalHandlePIDAction> goal_handle
)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Controller::handle_accepted
(
    const std::shared_ptr<GoalHandlePIDAction> goal_handle
)
{
    using namespace std::placeholders;
    // this needs to return quickly to avoid Controller::blocking the executor, so spin up a new thread
    std::thread{std::bind(&Controller::execute, this, _1), goal_handle}.detach();
}

bool Controller::areEqual(float float1, float float2, float epsilon)
{
    return (fabs(float1 - float2) < epsilon);
}

bool Controller::areEqual(std::vector<float>& current_state, std::vector<float>& desired_state)
{
    #define ORIENTATION_TOLERANCE 4.0f
    #define POSITION_TOLERANCE 0.06f

    bool equal = true;
    for (std::vector<float>::size_type i = 0; i < 3; i++)
    {
        if (!areEqual(current_state[i], desired_state[i], ORIENTATION_TOLERANCE)) //.05*current_state[i])
        {
            equal = false;
        }
    }
    for (std::vector<float>::size_type j = 3; j < 6; j++)
    {
        if (!areEqual(current_state[j], desired_state[j], POSITION_TOLERANCE)) //.05*current_state[i])
        {
            equal = false;
        }
    }
    return equal;
}

bool Controller::equalToZero(vector<int> thrustVect)
{
    bool equal = true;
    for (int thrust : thrustVect)
    {
        if (thrust > 1) {equal = false;}
    }
    return equal;
}

bool Controller::isSlewRateLow(int totalSlewRate)
{
    #define LOW_SLEW_VALUE 1;
    return totalSlewRate < LOW_SLEW_VALUE;
}

int Controller::calculateTotalSlew(deque<vector<int>>& slew_buffer)
{
    int slew_rate = 0;
    for (size_t i = 0; i < slew_buffer.size() - 1; i++)
    {   
        vector<int> difference = abs(slew_buffer[i]) - abs(slew_buffer[i+1]);
        slew_rate += abs(sum(difference));
    }
    return slew_rate;
}

void Controller::execute(const std::shared_ptr<GoalHandlePIDAction> goal_handle)
{
    /* Goal Initialization */
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(20);

    std::shared_ptr<PIDAction::Feedback> feedback = std::make_shared<PIDAction::Feedback>();
    std::shared_ptr<PIDAction::Result> result = std::make_shared<PIDAction::Result>();
    const auto goal = goal_handle->get_goal();

    this->desired_state_ = goal->desired_state;

    this->stabilize_robot_ = false;
    /* Init States */
    std::vector<float>& state = feedback->current_state;
    vector<float> thrusts = this->update_PID(this->current_state_, this->desired_state_);
    vector<int> thrustInts = this->make_CAN_request(thrusts);
    for (int thrust : thrustInts)
    {
        state.push_back((float)thrust);
    }

    int cycles_at_set_point = 0;
    deque<vector<int>> slew_buffer;
    bool slew_rate_low = false;
    /* Feedback Loop - Stop Conditions is that all Thrusts are close to zero*/
    while (!this->equalToZero(thrustInts) && (cycles_at_set_point <= 5) && !slew_rate_low)     {
        //   Check if there is a cancel request
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }
        this->areEqual(current_state_, desired_state_) ? cycles_at_set_point++ : cycles_at_set_point=0; 
        slew_buffer.push_front(thrustInts);
            if (slew_buffer.size() > 10) {
                slew_buffer.pop_back();
                slew_rate_low = isSlewRateLow(calculateTotalSlew(slew_buffer));
            }
        /* Update at Every Loop */
        thrusts = update_PID(this->current_state_, this->desired_state_);
        thrustInts = this->make_CAN_request(thrusts);
        for (int i = 0; i < thrustInts.size(); i++)
        { 
            state[i] = ((float)thrustInts[i]);
        }

        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
    }
    this->stabilize_robot_ = true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
                                        // CONTROL SERVICES // 
/////////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::stopRobot(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    this->desired_state_ = this->current_state_;
}

void Controller::pidReady()
{
    auto pid_ready_request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto pid_ready_future = this->pid_ready_client_->async_send_request(pid_ready_request);
}

void Controller::resetState()
{
    auto reset_state_request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto reset_state_future = this->reset_relative_state_client_->async_send_request(reset_state_request);
}

void Controller::resetPosition()
{     
    auto reset_position_request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto reset_position_future = this->reset_relative_position_client_->async_send_request(reset_position_request);
}

void Controller::usePosition(const  std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    this->use_position_ = request->data;
}

void Controller::stabilizeRobot(const   std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                            std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    this->stabilize_robot_ = request->data;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
                                    // SUBSCRIPTION CALLBACKS // 
/////////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::current_state_callback(const scion_types::msg::State::SharedPtr msg)
{
    if (!this->current_state_valid_) {this->current_state_valid_ = true;}
    this->current_state_= msg->state; 
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
                                    // GUI TUNING CONTROLS CALLBACKS // 
/////////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::kp_tuning_callback(const scion_types::msg::PidTuning::SharedPtr msg) 
{
    shared_ptr<PID_Controller> axis = this->controller_.controllers[axis_tuning_map_[msg->axis]];
    // float scale_factor = axis->angle_wrap ? .0001 : .001;
    // axis->set_gains(scale_factor * msg->data, axis->k_i, axis->k_d);

    axis->set_gains(msg->data, axis->k_i, axis->k_d);
}

void Controller::ki_tuning_callback(const scion_types::msg::PidTuning::SharedPtr msg) 
{
    shared_ptr<PID_Controller> axis = this->controller_.controllers[axis_tuning_map_[msg->axis]];
    // float scale_factor = axis->angle_wrap ? .0001 : .001;
    // axis->set_gains(axis->k_p, scale_factor * msg->data, axis->k_d);

    axis->set_gains(axis->k_i, msg->data, axis->k_d);
}

void Controller::kd_tuning_callback(const scion_types::msg::PidTuning::SharedPtr msg) 
{
    shared_ptr<PID_Controller> axis = this->controller_.controllers[axis_tuning_map_[msg->axis]];
    // float scale_factor = axis->angle_wrap ? .0001 : .001;
    // axis->set_gains(axis->k_p, axis->k_i, scale_factor * msg->data);

    axis->set_gains(axis->k_d, axis->k_i, msg->data);
}
