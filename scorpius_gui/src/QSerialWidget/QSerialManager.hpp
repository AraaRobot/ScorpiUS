#ifndef QSERIAL_MANAGER_HPP
#define QSERIAL_MANAGER_HPP

#include "QSerialWidget.hpp"
#include <rclcpp/rclcpp.hpp>
#include "scorpius_main/srv/serial_config.hpp"
#include "scorpius_main/msg/serial_heartbeat.hpp"
#include "scorpius_main/msg/serial_status.hpp"

class QSerialManager : public QWidget
{
    public:
        QSerialManager(std::shared_ptr<rclcpp::Node> node_, QWidget* parent);
        ~QSerialManager();

    std::shared_ptr<rclcpp::Node> _node;
};

#endif