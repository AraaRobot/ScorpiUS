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
#include "scorpius_main/srv/serial_ports.hpp"
#include "helpers/asyncExecutor.hpp"

class QSerialManager : public QWidget, public std::enable_shared_from_this<QSerialManager>
{
    Q_OBJECT

    static constexpr int ROWS = 2;
    static constexpr int COLS = 3;

  public:
  QSerialManager(std::shared_ptr<rclcpp::Node> node_, QWidget* parent);
  ~QSerialManager();

  private:
    void CB_subStatus(const scorpius_main::msg::SerialStatus& msg_);

    void writeMessage(const QString& message);

    std::array<QString, 32> last_message;
    int current_status_index = 0;
    int current_port_index = 0;
    bool change_connection = false;

    std::shared_ptr<rclcpp::Node> _node;

    QGridLayout* _grid;
    QComboBox* _box;
    QPushButton* _button;
    QPushButton* _refresh;
    QLabel* _label;
    QScrollArea* _scroll;

    AsyncExecutor _executor = AsyncExecutor();

    rclcpp::Client<scorpius_main::srv::SerialPorts>::SharedPtr _srv_ports;
    rclcpp::Subscription<scorpius_main::msg::SerialStatus>::SharedPtr _sub_status;
    rclcpp::Publisher<scorpius_main::msg::SerialHeartbeat>::SharedPtr _pub_heartbeat;
    rclcpp::Publisher<scorpius_main::msg::SerialStatus>::SharedPtr _pub_status;
    rclcpp::Client<scorpius_main::srv::SerialConfig>::SharedPtr _srv_config;

  signals:
        void serialPortsSignal(const std::vector<std::string>& port_name);
        void serialStatusSignal(const std::string& message, bool is_connected);
        void buttonFinishedSignal(const std::string& message);
    
  private slots:
        void serialPortsSlot(const std::vector<std::string>& port_name);
        void serialStatusSlot(const std::string& message, bool is_connected);
        void comboBoxIndexChanged(int index);
        void connectButtonClicked();
        void refreshButtonClicked();
        void buttonFinishedSlot(const std::string& message);
};

#endif