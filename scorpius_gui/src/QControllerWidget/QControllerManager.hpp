#ifndef QCONTROLLER_MANAGER_HPP
#define QCONTROLLER_MANAGER_HPP

#include "QControllerWidget.hpp"
#include <rclcpp/rclcpp.hpp>

class QControllerManager : public QWidget
{
    public:
        QControllerManager(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_ = nullptr);

    private:
        std::unique_ptr<QControllerWidget> _widget{nullptr};
        std::shared_ptr<rclcpp::Node> _node;
};

#endif