#ifndef SCORPIUS_COMM_HPP
#define SCORPIUS_COMM_HPP

#include <rclcpp/rclcpp.hpp>

class CommNode : public rclcpp::Node
{
  public:
    CommNode();
};

#endif  // SCORPIUS_COMM_HPP