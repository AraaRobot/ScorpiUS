#ifndef QCONTROLLER_MANAGER_HPP
#define QCONTROLLER_MANAGER_HPP

#include "QControllerWidget.hpp"

#include "helpers/asyncExecutor.hpp"
#include "scorpius_main/srv/joy_config.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>

class QControllerManager : public QWidget
{
  private:
    static constexpr double JOYSTICK_DEADZONE_DEFAULT = 0.05;
    static constexpr const char* SERVICE_NAME = "/scorpius/joy_config";

  public:
    QControllerManager(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_ = nullptr);
    ~QControllerManager();

  private:
    void setupUI();
    void setupLayout();
    void setDeadzone();
    void setControllerType(const std::string& type_);

    QControllerWidget* _widget{nullptr};
    QVBoxLayout* _verticalLayout{nullptr};
    QHBoxLayout* _horizontalLayout{nullptr};
    QDoubleSpinBox* _deadzoneBox{nullptr};
    QComboBox* _controllerSelectBox{nullptr};
    QLabel* _deadzoneLabel{nullptr};
    QLabel* _controllerSelectLabel{nullptr};

    AsyncExecutor _executor = AsyncExecutor();

    std::shared_ptr<rclcpp::Node> _node;
    rclcpp::Client<scorpius_main::srv::JoyConfig>::SharedPtr _client;
};

#endif