#ifndef SCORPIUS_JOY_HPP
#define SCORPIUS_JOY_HPP

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joy.hpp"
#include "scorpius_main/msg/joy_format.hpp"
#include <cstdint>

class JoyFormator : public rclcpp::Node
{
  public:
    JoyFormator();

  private:
    void joySubscriber_CB(const sensor_msgs::msg::Joy& joyInput_);
    void joyPublisher_CB();

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _sub_joy;
    rclcpp::Publisher<scorpius_main::msg::JoyFormat>::SharedPtr _pub_joyFormat;
    rclcpp::TimerBase::SharedPtr _timer_pub;

    sensor_msgs::msg::Joy _lastMsg;
};

enum class eKeybinding : uint8_t
{
    a = 0,
    b,
    x,
    y,
    l1,
    r1,
    joystick_left_push,
    joystick_right_push,
    ext0,
    ext1,
    ext2,
    joystick_left_side,
    joystick_left_front,
    l2,
    joystick_right_side,
    joystick_right_front,
    r2,
    cross_front,
    cross_side,
    eKeybinding_END
};

struct sControllerConfig
{
    int8_t buttons[std::to_underlying(eKeybinding::eKeybinding_END)] = {-1};
    int8_t axes[std::to_underlying(eKeybinding::eKeybinding_END)] = {-1};

    float trigger_range_min = -1.0f;
    float trigger_range_max = 1.0f;

    float joystick_dead_zone = 0.0f;

    // Point this pointer to a function for a controller which needs
    // specific custom execution each publish loop. This can be used to
    // handle a weird deconnection from controller
    // void (JoyFormator::*custom_steps)(rover_msgs::msg::Joy* formatted_joy);
};

#endif  // SCORPIUS_JOY_HPP