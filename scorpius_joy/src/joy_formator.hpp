#ifndef SCORPIUS_JOY_HPP
#define SCORPIUS_JOY_HPP

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joy.hpp"
#include "scorpius_main/msg/joy.hpp"
#include "scorpius_main/srv/joy_config.hpp"
#include <cstdint>
#include <array>
#include <rclcpp/logger.hpp>

enum class eKeybinding : uint8_t
{
    a = 0,
    b,
    x,
    y,
    l1,
    r1,
    l3,
    r3,
    share,
    opts,
    home,
    joystick_left_horiz,
    joystick_left_vert,
    l2,
    joystick_right_horiz,
    joystick_right_vert,
    r2,
    cross_vert,
    cross_horiz,
    eKeybinding_END
};

struct sControllerConfig
{
    std::string controllerType = "";
    int8_t buttons[std::to_underlying(eKeybinding::eKeybinding_END)] = {-1};
    int8_t axes[std::to_underlying(eKeybinding::eKeybinding_END)] = {-1};

    float trigger_range_min = -1.0f;
    float trigger_range_max = 1.0f;

    float joystick_dead_zone = 0.0f;

    sControllerConfig()
    {
      for (int i = 0; i < std::to_underlying(eKeybinding::eKeybinding_END); i++)
      {
        buttons[i] = -1;
        axes[i] = -1;
      }
    }

    // Point this pointer to a function for a controller which needs
    // specific custom execution each publish loop. This can be used to
    // handle a weird deconnection from controller
    // void (JoyFormator::*custom_steps)(rover_msgs::msg::Joy* formatted_joy);
};

class JoyFormator : public rclcpp::Node
{
  static constexpr const char* DEFAULT_CONTROLLER = "DS5";
  static constexpr const char* TOPIC_SUBSCRIBER_NAME = "raw/joy";  // Relative: remapping works
  static constexpr const char* TOPIC_PUBLISHER_NAME = "/scorpius/joy";  // Absolute: no namespace prefix
  static constexpr const char* SERVICE_NAME = "/scorpius/joy_config";  // Absolute: no namespace prefix

  public:
    JoyFormator();

  private:
    void joySubscriber_CB(const sensor_msgs::msg::Joy& joyInput_);
    void joyPublisher_CB();

    bool setControllerType(std::string controllerName_);
    void setControllerType(std::string controllerName_, scorpius_main::srv::JoyConfig::Response& response_);
    void getControllerType(scorpius_main::srv::JoyConfig::Response& response_);
    void setDeadzone(float deadzone_, scorpius_main::srv::JoyConfig::Response& response_);
    void getDeadzone(scorpius_main::srv::JoyConfig::Response& response_);
    
    template<typename T>
    T getJoyValue(eKeybinding key_);
    float applyJoystickDeadzone(eKeybinding joystick_);
    float getTriggerValues(eKeybinding trigger_);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _sub_joy;
    rclcpp::Publisher<scorpius_main::msg::Joy>::SharedPtr _pub_joyFormat;
    rclcpp::TimerBase::SharedPtr _timer_pub;
    rclcpp::Service<scorpius_main::srv::JoyConfig>::SharedPtr _srv_config;

    sensor_msgs::msg::Joy _currentMsg;
    sensor_msgs::msg::Joy _lastMsg;
    scorpius_main::msg::Joy _lastFormattedJoy;

    sControllerConfig _currentConfig;
    bool isControllerConnected = false;
};

#endif  // SCORPIUS_JOY_HPP