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
    if (!this->controllerConnected)
        this->controllerConnected = true;
    this->_currentMsg = msg_;
}

void JoyFormator::joyPublisher_CB(void)
{
    if (!this->controllerConnected)
    {
        return;
    }

    if (_lastMsg == _currentMsg || _currentMsg.buttons.size() == 0 || _currentMsg.axes.size() == 0)
    {
        _pub_joyFormat->publish(_lastFormattedJoy);
    }

    scorpius_main::msg::Joy msg;
    msg.joy_data[scorpius_main::msg::Joy::A] = this->getJoyValue<bool>(eKeybinding::a);
    msg.joy_data[scorpius_main::msg::Joy::B] = this->getJoyValue<bool>(eKeybinding::b);
    msg.joy_data[scorpius_main::msg::Joy::X] = this->getJoyValue<bool>(eKeybinding::x);
    msg.joy_data[scorpius_main::msg::Joy::Y] = this->getJoyValue<bool>(eKeybinding::y);
    msg.joy_data[scorpius_main::msg::Joy::L1] = this->getJoyValue<bool>(eKeybinding::l1);
    msg.joy_data[scorpius_main::msg::Joy::R1] = this->getJoyValue<bool>(eKeybinding::r1);

    if (this->getJoyValue<int8_t>(eKeybinding::cross_vert) > 0)
    {
        msg.joy_data[scorpius_main::msg::Joy::CROSS_UP] = true;
        msg.joy_data[scorpius_main::msg::Joy::CROSS_DOWN] = false;
    }
    else
    {
        msg.joy_data[scorpius_main::msg::Joy::CROSS_UP] = false;
        msg.joy_data[scorpius_main::msg::Joy::CROSS_DOWN] = true;
    }

    if (this->getJoyValue<int8_t>(eKeybinding::cross_horiz) > 0)
    {
        msg.joy_data[scorpius_main::msg::Joy::CROSS_LEFT] = true;
        msg.joy_data[scorpius_main::msg::Joy::CROSS_RIGHT] = false;
    }
    else
    {
        msg.joy_data[scorpius_main::msg::Joy::CROSS_LEFT] = false;
        msg.joy_data[scorpius_main::msg::Joy::CROSS_RIGHT] = true;
    }
    // msg.joy_data[scorpius_main::msg::Joy::L2] = this->_currentMsg.buttons[5];
    // msg.joy_data[scorpius_main::msg::Joy::R2] = this->_currentMsg.buttons[7];
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
    if (controllerName == "DS5") // DualSense 5 controller
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
    // else {}
}
