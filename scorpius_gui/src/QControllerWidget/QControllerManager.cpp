#include "QControllerManager.hpp"

QControllerManager::QControllerManager(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_ = nullptr) 
{
    _widget = std::make_unique<QControllerWidget>(node_, this);


}