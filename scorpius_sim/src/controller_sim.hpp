#ifndef CONTROLLER_SIM_HPP
#define CONTROLLER_SIM_HPP

#include <rclcpp/rclcpp.hpp>
#include <cstdint>

#include "scorpius_main/msg/joy.hpp"

enum class eSimState : uint8_t
{
    FORWARD = 0U,
    BACKWARD,
    LEFT,
    RIGHT
};

class ControllerSim : public rclcpp::Node
{
    static constexpr const char* TOPIC_JOY_SIM = "/scorpius/joy";
    static constexpr int64_t PUB_FREQ = 10;

  public:
    ControllerSim();

  private:
    void simControllerPub_CB();
    void updateState();
    const char* getCurrentState();

    rclcpp::Publisher<scorpius_main::msg::Joy>::SharedPtr _pub_controllerSim;
    rclcpp::TimerBase::SharedPtr _timer_pub;
    rclcpp::TimerBase::SharedPtr _timer_state;

    scorpius_main::msg::Joy _lastSentMessage;

    eSimState _currentState = eSimState::FORWARD;
};

#endif  // CONTROLLER_SIM_HPP