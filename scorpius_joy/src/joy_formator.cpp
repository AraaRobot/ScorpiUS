#include "joy_formator.hpp"

static constexpr int64_t PUB_FREQ = 10;

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
    _sub_joy = this->create_subscription<sensor_msgs::msg::Joy>("raw/joy",
                                                                10,
                                                                [this](const sensor_msgs::msg::Joy msg_)
                                                                {
                                                                    this->joySubscriber_CB(msg_);
                                                                });

    _pub_joyFormat = this->create_publisher<scorpius_main::msg::Joy>("/scorpius/joy", 10);

    _timer_pub = this->create_wall_timer(std::chrono::milliseconds(static_cast<size_t>(1000 / PUB_FREQ)),
                                         [this](void)
                                         {
                                             this->joyPublisher_CB();
                                         });
}

void JoyFormator::joySubscriber_CB(const sensor_msgs::msg::Joy& msg_)
{
    if (!this->isControllerConnected)
        this->isControllerConnected = true;
    this->_currentMsg = msg_;
}

void JoyFormator::joyPublisher_CB(void)
{
    if (!this->isControllerConnected)
    {
        RCLCPP_ERROR(this->get_logger(), "Controller not connected");
        return;
    }

    if (_lastMsg == _currentMsg || _currentMsg.buttons.size() == 0 || _currentMsg.axes.size() == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid msg received. Sending last formatted msg");
        _pub_joyFormat->publish(_lastFormattedJoy);
    }
    scorpius_main::msg::Joy msg;
    msg.joy_data[scorpius_main::msg::Joy::A] = this->getJoyValue<bool>(eKeybinding::a);
    msg.joy_data[scorpius_main::msg::Joy::B] = this->getJoyValue<bool>(eKeybinding::b);
    msg.joy_data[scorpius_main::msg::Joy::X] = this->getJoyValue<bool>(eKeybinding::x);
    msg.joy_data[scorpius_main::msg::Joy::Y] = this->getJoyValue<bool>(eKeybinding::y);
    msg.joy_data[scorpius_main::msg::Joy::L1] = this->getJoyValue<bool>(eKeybinding::l1);
    msg.joy_data[scorpius_main::msg::Joy::L3] = this->getJoyValue<bool>(eKeybinding::l3);
    msg.joy_data[scorpius_main::msg::Joy::R1] = this->getJoyValue<bool>(eKeybinding::r1);
    msg.joy_data[scorpius_main::msg::Joy::R3] = this->getJoyValue<bool>(eKeybinding::r3);
    msg.joy_data[scorpius_main::msg::Joy::SHARE] = this->getJoyValue<bool>(eKeybinding::share);
    msg.joy_data[scorpius_main::msg::Joy::OPTS] = this->getJoyValue<bool>(eKeybinding::opts);
    msg.joy_data[scorpius_main::msg::Joy::HOME] = this->getJoyValue<bool>(eKeybinding::home);

    float crossTemp = this->getJoyValue<float>(eKeybinding::cross_vert);
    msg.joy_data[scorpius_main::msg::Joy::CROSS_UP] = (crossTemp > 0.0f) ? 1.0f : 0.0f;
    msg.joy_data[scorpius_main::msg::Joy::CROSS_DOWN] = (crossTemp < 0.0f) ? 1.0f : 0.0f;

    crossTemp = this->getJoyValue<float>(eKeybinding::cross_horiz);
    msg.joy_data[scorpius_main::msg::Joy::CROSS_LEFT] = (crossTemp > 0.0f) ? 1.0f : 0.0f;
    msg.joy_data[scorpius_main::msg::Joy::CROSS_RIGHT] = (crossTemp < 0.0f) ? 1.0f : 0.0f;

    msg.joy_data[scorpius_main::msg::Joy::JOYSTICK_LEFT_HORIZ] = this->applyJoystickDeadzone(eKeybinding::joystick_left_horiz);
    msg.joy_data[scorpius_main::msg::Joy::JOYSTICK_LEFT_VERT] = this->applyJoystickDeadzone(eKeybinding::joystick_left_vert);
    msg.joy_data[scorpius_main::msg::Joy::JOYSTICK_RIGHT_HORIZ] = this->applyJoystickDeadzone(eKeybinding::joystick_right_horiz);
    msg.joy_data[scorpius_main::msg::Joy::JOYSTICK_RIGHT_VERT] = this->applyJoystickDeadzone(eKeybinding::joystick_right_vert);
    _pub_joyFormat->publish(msg);
    _lastFormattedJoy = msg;
}

template<typename T>
T JoyFormator::getJoyValue(eKeybinding key)
{
    if (_currentConfig.buttons[std::to_underlying(key)] != -1)
    {
        return static_cast<T>(_currentMsg.buttons[_currentConfig.buttons[std::to_underlying(key)]]);
    }
    else if (_currentConfig.axes[std::to_underlying(key)] != -1)
    {
        return static_cast<T>(_currentMsg.axes[_currentConfig.axes[std::to_underlying(key)]]);
    }

    return static_cast<T>(0.0f);
}

void JoyFormator::setControllerType(std::string controllerName)
{
    if (controllerName == "DS5")  // DualSense 5 controller
    {
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
    }
    // else {} // Add more configurations here...
}

float JoyFormator::applyJoystickDeadzone(eKeybinding joystick)
{
    float joyValue = this->getJoyValue<float>(joystick);
    float sign = (joyValue >= 0.0f) ? 1.0f : -1.0f;
    float absValue = std::abs(joyValue);

    if (absValue < this->_currentConfig.joystick_dead_zone)
    {
        return 0.0f;
    }

    // Remap from [deadzone, 1] to [0, 1]
    absValue = (absValue - this->_currentConfig.joystick_dead_zone) / (1.0f - this->_currentConfig.joystick_dead_zone);
    absValue = std::min(1.0f, absValue);  // Clamp to 1.0

    return sign * absValue;
}
