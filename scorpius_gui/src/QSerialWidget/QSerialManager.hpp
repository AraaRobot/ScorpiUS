#ifndef QSERIAL_MANAGER_HPP
#define QSERIAL_MANAGER_HPP

#include <QWidget>
#include <QGridLayout>
#include <QComboBox>
#include <QRadioButton>
// #include <QTextEdit>
#include <QLabel>
#include <QLineEdit>

#include "QSerialWidget.hpp"
#include <rclcpp/rclcpp.hpp>
#include "scorpius_main/srv/serial_config.hpp"
#include "scorpius_main/msg/serial_heartbeat.hpp"
#include "scorpius_main/msg/serial_status.hpp"

class QSerialManager : public QWidget
{
    static constexpr int ROWS = 2;
    static constexpr int COLS = 1;

  public:
    QSerialManager(std::shared_ptr<rclcpp::Node> node_, QWidget* parent);
    ~QSerialManager();

    std::shared_ptr<rclcpp::Node> _node;

    QGridLayout* _grid;
    QComboBox* _box;
    QRadioButton* _button;
    QLabel* _label;

    rclcpp::Publisher<scorpius_main::msg::SerialHeartbeat>::SharedPtr _pub_heartbeat;
    rclcpp::Publisher<scorpius_main::msg::SerialStatus>::SharedPtr _pub_status;
    rclcpp::Service<scorpius_main::srv::SerialConfig>::SharedPtr _srv_config;
};

#endif