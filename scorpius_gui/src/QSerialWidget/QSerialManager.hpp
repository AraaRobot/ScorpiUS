#ifndef QSERIAL_MANAGER_HPP
#define QSERIAL_MANAGER_HPP

#include <QWidget>
#include <QGridLayout>
#include <QComboBox>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>
#include <QString>
#include <QScrollArea>

#include <rclcpp/rclcpp.hpp>
#include "scorpius_main/srv/serial_config.hpp"
#include "scorpius_main/msg/serial_heartbeat.hpp"
#include "scorpius_main/msg/serial_status.hpp"
#include "scorpius_main/msg/serial_ports.hpp"

class QSerialManager : public QWidget
{
    Q_OBJECT

    static constexpr int ROWS = 2;
    static constexpr int COLS = 2;

  public:
  QSerialManager(std::shared_ptr<rclcpp::Node> node_, QWidget* parent);
  ~QSerialManager();

  private:
    void CB_subPorts(const scorpius_main::msg::SerialPorts& msg_);
    void CB_subStatus(const scorpius_main::msg::SerialStatus& msg_);

    QString last_message = "";
    bool change_connection = false;

    std::shared_ptr<rclcpp::Node> _node;

    QGridLayout* _grid;
    QComboBox* _box;
    QPushButton* _button;
    QLabel* _label;
    QScrollArea* _scroll;

    rclcpp::Subscription<scorpius_main::msg::SerialPorts>::SharedPtr _sub_ports;
    rclcpp::Subscription<scorpius_main::msg::SerialStatus>::SharedPtr _sub_status;
    rclcpp::Publisher<scorpius_main::msg::SerialHeartbeat>::SharedPtr _pub_heartbeat;
    rclcpp::Publisher<scorpius_main::msg::SerialStatus>::SharedPtr _pub_status;
    rclcpp::Client<scorpius_main::srv::SerialConfig>::SharedPtr _srv_config;

  signals:
        void serialPortsSignal(std::vector<std::string> port_name);
        void serialStatusSignal(std::string message, bool is_connected);
    
  private slots:
        void serialPortsSlot(std::vector<std::string> port_name);
        void serialStatusSlot(std::string message, bool is_connected);
        void comboBoxIndexChanged(int index);
        void pushButtonClicked();
};

#endif