#ifndef QCONTROLLER_MANAGER_HPP
#define QCONTROLLER_MANAGER_HPP

#include "QControllerWidget.hpp"
#include <rclcpp/rclcpp.hpp>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QComboBox>

class QControllerManager : public QWidget
{
  private:
    static constexpr double JOYSTICK_DEADZONE_MIN = 0.20;

  public:
    QControllerManager(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_ = nullptr);
    ~QControllerManager();

  private:
    void setupDeadzoneUI();
    void setupLayout();
    
    QControllerWidget* _widget{nullptr};
    QVBoxLayout* _verticalLayout{nullptr};
    QHBoxLayout* _horizontalLayout{nullptr};
    QDoubleSpinBox* _deadzoneLBox{nullptr};
    QDoubleSpinBox* _deadzoneRBox{nullptr};
    QLabel* _deadzoneLLabel{nullptr};
    QLabel* _deadzoneRLabel{nullptr};
    std::shared_ptr<rclcpp::Node> _node;
};

#endif