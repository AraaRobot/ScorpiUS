#include "controller_sim.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<ControllerSim> node = std::make_shared<ControllerSim>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

ControllerSim::ControllerSim():
    Node("ControllerSim")
{
    this->_pub_controllerSim = this->create_publisher<scorpius_main::msg::Joy>(this->TOPIC_JOY_SIM, 10);

    this->_timer_pub = this->create_wall_timer(std::chrono::milliseconds(static_cast<int64_t>(1000 / this->PUB_FREQ)),
                                               [this]()
                                               {
                                                   this->simControllerPub_CB();
                                               });

    this->_timer_state = this->create_wall_timer(std::chrono::seconds(this->STATE_CHANGE_TIME),
                                                 [this]()
                                                 {
                                                     this->updateState();
                                                 });

    RCLCPP_INFO(this->get_logger(), "Current state: %s", this->getCurrentState());
}

void ControllerSim::simControllerPub_CB()
{
    switch (this->_currentState)
    {
        case eSimState::FORWARD:
            this->_lastSentMessage.joy_data[scorpius_main::msg::Joy::JOYSTICK_LEFT_VERT] = 1.0f;
            this->_lastSentMessage.joy_data[scorpius_main::msg::Joy::JOYSTICK_LEFT_HORIZ] = 0.0f;
            break;
        case eSimState::BACKWARD:
            this->_lastSentMessage.joy_data[scorpius_main::msg::Joy::JOYSTICK_LEFT_VERT] = -1.0f;
            this->_lastSentMessage.joy_data[scorpius_main::msg::Joy::JOYSTICK_LEFT_HORIZ] = 0.0f;
            break;
        case eSimState::LEFT:
            this->_lastSentMessage.joy_data[scorpius_main::msg::Joy::JOYSTICK_LEFT_VERT] = 0.0f;
            this->_lastSentMessage.joy_data[scorpius_main::msg::Joy::JOYSTICK_LEFT_HORIZ] = 1.0f;
            break;
        case eSimState::RIGHT:
            this->_lastSentMessage.joy_data[scorpius_main::msg::Joy::JOYSTICK_LEFT_VERT] = 0.0f;
            this->_lastSentMessage.joy_data[scorpius_main::msg::Joy::JOYSTICK_LEFT_HORIZ] = -1.0f;
            break;
    }

    this->_pub_controllerSim->publish(this->_lastSentMessage);
}

void ControllerSim::updateState()
{
    switch (this->_currentState)
    {
        case eSimState::FORWARD:
            this->_currentState = eSimState::BACKWARD;
            break;
        case eSimState::BACKWARD:
            this->_currentState = eSimState::LEFT;
            break;
        case eSimState::LEFT:
            this->_currentState = eSimState::RIGHT;
            break;
        case eSimState::RIGHT:
            this->_currentState = eSimState::FORWARD;
            break;
    }
    RCLCPP_INFO(this->get_logger(), "Current controller simulation state: %s", this->getCurrentState());
}

const char* ControllerSim::getCurrentState()
{
    switch (this->_currentState)
    {
        case eSimState::FORWARD:
            return "Forward";
        case eSimState::BACKWARD:
            return "Backward";
        case eSimState::LEFT:
            return "Left";
        case eSimState::RIGHT:
            return "Right";
        default:
            return "Unknown";
    }
}