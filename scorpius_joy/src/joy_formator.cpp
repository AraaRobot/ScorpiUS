#include "joy_formator.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<JoyFormator> node = std::make_shared<JoyFormator>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

JoyFormator::JoyFormator():
    Node("JoyFormator")
{
    this->setControllerType(DEFAULT_CONTROLLER);
    _sub_joy = this->create_subscription<sensor_msgs::msg::Joy>(TOPIC_SUBSCRIBER_NAME,
                                                                10,
                                                                [this](const sensor_msgs::msg::Joy& msg_)
                                                                {
                                                                    this->joySubscriber_CB(msg_);
                                                                });

    _pub_joyFormat = this->create_publisher<scorpius_main::msg::Joy>(TOPIC_PUBLISHER_NAME, 10);

    _timer_pub = this->create_wall_timer(std::chrono::milliseconds(static_cast<size_t>(1000 / PUB_FREQ)),
                                         [this](void)
                                         {
                                             this->joyPublisher_CB();
                                         });

    _srv_config = this->create_service<scorpius_main::srv::JoyConfig>(
        SERVICE_NAME,
        [this](const std::shared_ptr<scorpius_main::srv::JoyConfig::Request> request_,
               std::shared_ptr<scorpius_main::srv::JoyConfig::Response> response_)
        {
            if (!request_ || !response_)
            {
                RCLCPP_ERROR(this->get_logger(), "NULL request or response received.");
                return;
            }

            switch (request_->command)
            {
                case scorpius_main::srv::JoyConfig::Request::SET_DEADZONE:
                    this->setDeadzone(request_->deadzone, *response_);
                    break;
                case scorpius_main::srv::JoyConfig::Request::GET_DEADZONE:
                    this->getDeadzone(*response_);
                    break;
                case scorpius_main::srv::JoyConfig::Request::SET_TYPE:
                    this->setControllerType(request_->type, *response_);
                    break;
                case scorpius_main::srv::JoyConfig::Request::GET_TYPE:
                    this->getControllerType(*response_);
                    break;
                default:
                    response_->success = false;
                    response_->response = "Unrecognized command for JoyConfig service";
                    break;
            }
        });
}

void JoyFormator::joySubscriber_CB(const sensor_msgs::msg::Joy& msg_)
{
    std::lock_guard<std::mutex> lock(mutex);
    if (!this->_isControllerConnected)
        this->_isControllerConnected = true;
    this->_currentMsg = msg_;
}

void JoyFormator::joyPublisher_CB(void)
{
    sensor_msgs::msg::Joy currentMsg;
    sensor_msgs::msg::Joy lastMsg;
    scorpius_main::msg::Joy lastFormattedJoy;
    sControllerConfig config;
    bool isControllerConnected = false;

    {
        std::lock_guard<std::mutex> lock(mutex);
        currentMsg = this->_currentMsg;
        lastMsg = this->_lastMsg;
        isControllerConnected = this->_isControllerConnected;
        config = this->_currentConfig;
        lastFormattedJoy = this->_lastFormattedJoy;
    }

    if (!isControllerConnected)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), THROTTLE_DELAY, "Controller not connected");
        return;
    }

    if (currentMsg == lastMsg || currentMsg.buttons.size() == 0 || currentMsg.axes.size() == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid msg received. Sending last formatted msg");
        _pub_joyFormat->publish(lastFormattedJoy);
        return;
    }

    scorpius_main::msg::Joy msg;
    msg.joy_data[scorpius_main::msg::Joy::A] = this->getJoyValue<bool>(eKeybinding::a, currentMsg, config);
    msg.joy_data[scorpius_main::msg::Joy::B] = this->getJoyValue<bool>(eKeybinding::b, currentMsg, config);
    msg.joy_data[scorpius_main::msg::Joy::X] = this->getJoyValue<bool>(eKeybinding::x, currentMsg, config);
    msg.joy_data[scorpius_main::msg::Joy::Y] = this->getJoyValue<bool>(eKeybinding::y, currentMsg, config);
    msg.joy_data[scorpius_main::msg::Joy::L1] = this->getJoyValue<bool>(eKeybinding::l1, currentMsg, config);
    msg.joy_data[scorpius_main::msg::Joy::L3] = this->getJoyValue<bool>(eKeybinding::l3, currentMsg, config);
    msg.joy_data[scorpius_main::msg::Joy::R1] = this->getJoyValue<bool>(eKeybinding::r1, currentMsg, config);
    msg.joy_data[scorpius_main::msg::Joy::R3] = this->getJoyValue<bool>(eKeybinding::r3, currentMsg, config);
    msg.joy_data[scorpius_main::msg::Joy::SHARE] = this->getJoyValue<bool>(eKeybinding::share, currentMsg, config);
    msg.joy_data[scorpius_main::msg::Joy::OPTS] = this->getJoyValue<bool>(eKeybinding::opts, currentMsg, config);
    msg.joy_data[scorpius_main::msg::Joy::HOME] = this->getJoyValue<bool>(eKeybinding::home, currentMsg, config);

    msg.joy_data[scorpius_main::msg::Joy::L2] = this->getTriggerValues(eKeybinding::l2, currentMsg, config);
    msg.joy_data[scorpius_main::msg::Joy::R2] = this->getTriggerValues(eKeybinding::r2, currentMsg, config);

    float crossTemp = this->getJoyValue<float>(eKeybinding::cross_vert, currentMsg, config);
    msg.joy_data[scorpius_main::msg::Joy::CROSS_UP] = (crossTemp > 0.0f) ? 1.0f : 0.0f;
    msg.joy_data[scorpius_main::msg::Joy::CROSS_DOWN] = (crossTemp < 0.0f) ? 1.0f : 0.0f;

    crossTemp = this->getJoyValue<float>(eKeybinding::cross_horiz, currentMsg, config);
    msg.joy_data[scorpius_main::msg::Joy::CROSS_LEFT] = (crossTemp > 0.0f) ? 1.0f : 0.0f;
    msg.joy_data[scorpius_main::msg::Joy::CROSS_RIGHT] = (crossTemp < 0.0f) ? 1.0f : 0.0f;

    msg.joy_data[scorpius_main::msg::Joy::JOYSTICK_LEFT_HORIZ]
        = this->applyJoystickDeadzone(eKeybinding::joystick_left_horiz, currentMsg, config);
    msg.joy_data[scorpius_main::msg::Joy::JOYSTICK_LEFT_VERT]
        = this->applyJoystickDeadzone(eKeybinding::joystick_left_vert, currentMsg, config);
    msg.joy_data[scorpius_main::msg::Joy::JOYSTICK_RIGHT_HORIZ]
        = this->applyJoystickDeadzone(eKeybinding::joystick_right_horiz, currentMsg, config);
    msg.joy_data[scorpius_main::msg::Joy::JOYSTICK_RIGHT_VERT]
        = this->applyJoystickDeadzone(eKeybinding::joystick_right_vert, currentMsg, config);
    _pub_joyFormat->publish(msg);

    {
        std::lock_guard<std::mutex> lock(mutex);
        _lastFormattedJoy = msg;
        _lastMsg = _currentMsg;
    }
}

template<typename T>
T JoyFormator::getJoyValue(const eKeybinding key_, const sensor_msgs::msg::Joy& msg_, const sControllerConfig& config_)
{
    if (config_.buttons[std::to_underlying(key_)] != -1)
    {
        return static_cast<T>(msg_.buttons[config_.buttons[std::to_underlying(key_)]]);
    }
    else if (config_.axes[std::to_underlying(key_)] != -1)
    {
        return static_cast<T>(msg_.axes[config_.axes[std::to_underlying(key_)]]);
    }

    return static_cast<T>(0.0f);
}

bool JoyFormator::setControllerType(const std::string& controllerName_)
{
    if (controllerName_ == "DS5")  // DualSense 5 controller
    {
        std::lock_guard<std::mutex> lock(mutex);
        _currentConfig.controllerType = "DS5";
        _currentConfig.buttons[std::to_underlying(eKeybinding::a)] = 0;
        _currentConfig.buttons[std::to_underlying(eKeybinding::b)] = 1;
        _currentConfig.buttons[std::to_underlying(eKeybinding::y)] = 2;
        _currentConfig.buttons[std::to_underlying(eKeybinding::x)] = 3;
        _currentConfig.buttons[std::to_underlying(eKeybinding::l1)] = 4;
        _currentConfig.buttons[std::to_underlying(eKeybinding::r1)] = 5;
        _currentConfig.buttons[std::to_underlying(eKeybinding::share)] = 8;
        _currentConfig.buttons[std::to_underlying(eKeybinding::opts)] = 9;
        _currentConfig.buttons[std::to_underlying(eKeybinding::home)] = 10;
        _currentConfig.buttons[std::to_underlying(eKeybinding::l3)] = 11;
        _currentConfig.buttons[std::to_underlying(eKeybinding::r3)] = 12;

        _currentConfig.axes[std::to_underlying(eKeybinding::joystick_left_horiz)] = 0;
        _currentConfig.axes[std::to_underlying(eKeybinding::joystick_left_vert)] = 1;
        _currentConfig.axes[std::to_underlying(eKeybinding::joystick_right_horiz)] = 3;
        _currentConfig.axes[std::to_underlying(eKeybinding::joystick_right_vert)] = 4;
        _currentConfig.axes[std::to_underlying(eKeybinding::l2)] = 2;
        _currentConfig.axes[std::to_underlying(eKeybinding::r2)] = 5;
        _currentConfig.axes[std::to_underlying(eKeybinding::cross_horiz)] = 6;
        _currentConfig.axes[std::to_underlying(eKeybinding::cross_vert)] = 7;

        _currentConfig.joystick_dead_zone = 0.075f;
        _currentConfig.trigger_range_min = 0.0f;
        _currentConfig.trigger_range_max = 1.0f;

        RCLCPP_INFO(this->get_logger(), "Controller type now set to DS5");
        return true;
    }
    // else if {} // Add more configurations here...

    RCLCPP_ERROR(this->get_logger(), "Unrecognized controller type. Default to DS5");
    this->setControllerType("DS5");
    return false;
}

void JoyFormator::setControllerType(const std::string& controllerName_, scorpius_main::srv::JoyConfig::Response& response_)
{
    bool success = this->setControllerType(controllerName_);
    response_.success = success;
    response_.response
        = success ? "Successfully changed controller type to " + controllerName_ : "Unrecognized controller type. Default to DS5";
}

void JoyFormator::getControllerType(scorpius_main::srv::JoyConfig::Response& response_)
{
    response_.success = true;
    {
        std::lock_guard<std::mutex> lock(mutex);
        response_.response = this->_currentConfig.controllerType;
    }
}

void JoyFormator::setDeadzone(float deadzone_, scorpius_main::srv::JoyConfig::Response& response_)
{
    float deadzone = std::clamp(deadzone_, 0.0f, 1.0f);
    std::string msg = "Deadzone now set to " + std::to_string(deadzone);

    {
        const std::lock_guard<std::mutex> lock(mutex);
        this->_currentConfig.joystick_dead_zone = deadzone;
    }
    response_.success = true;
    response_.response = msg;
    RCLCPP_INFO(this->get_logger(), msg.c_str());
}

void JoyFormator::getDeadzone(scorpius_main::srv::JoyConfig::Response& response_)
{
    response_.success = true;
    {
        std::lock_guard<std::mutex> lock(mutex);
        response_.deadzone = this->_currentConfig.joystick_dead_zone;
    }
}

float JoyFormator::applyJoystickDeadzone(const eKeybinding joystick_,
                                         const sensor_msgs::msg::Joy& msg_,
                                         const sControllerConfig& config_)
{
    if (config_.joystick_dead_zone >= 1.0f)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(),
                             *this->get_clock(),
                             THROTTLE_DELAY,
                             "Deadzone currently set to 1.0. Unable to properly get joystick value");
        return 0.0f;
    }

    float joyValue = this->getJoyValue<float>(joystick_, msg_, config_);
    float sign = (joyValue >= 0.0f) ? 1.0f : -1.0f;
    float absValue = std::abs(joyValue);

    if (absValue < config_.joystick_dead_zone)
    {
        return 0.0f;
    }

    // Remap from [deadzone, 1] to [0, 1]
    absValue = (absValue - config_.joystick_dead_zone) / (1.0f - config_.joystick_dead_zone);
    absValue = std::clamp(absValue, 0.0f, 1.0f);

    return sign * absValue;
}

float JoyFormator::getTriggerValues(const eKeybinding trigger_,
                                    const sensor_msgs::msg::Joy& msg_,
                                    const sControllerConfig& config_)
{
    // Normal trigger values are from 1 (released) to -1 (pressed)
    float trigValue = this->getJoyValue<float>(trigger_, msg_, config_);
    float trigMax = config_.trigger_range_max;
    float trigMin = config_.trigger_range_min;
    //(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    // Remap from [-1, 1] to [0, 1]
    trigValue = (trigValue - 1.0f) * (trigMax - trigMin) / (-2.0f) + trigMin;

    return std::clamp(trigValue, 0.0f, 1.0f);
}
