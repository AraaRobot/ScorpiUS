#ifndef QSERIAL_MANAGER_HPP
#define QSERIAL_MANAGER_HPP

#include "helpers/asyncExecutor.hpp"
#include "scorpius_main/srv/serial_config.hpp"
#include "scorpius_main/msg/serial_heartbeat.hpp"
#include "scorpius_main/msg/serial_status.hpp"
#include "scorpius_main/msg/state_changed.hpp"
#include "scorpius_main/srv/serial_ports.hpp"
#include "scorpius_main/srv/controller_state.hpp"

#include <rclcpp/rclcpp.hpp>

#include <QComboBox>
#include <QGridLayout>
#include <QLineEdit>
#include <QLabel>
#include <QPointer>
#include <QPushButton>
#include <QString>
#include <QTextEdit>
#include <QWidget>

#include <array>
#include <string>
#include <vector>

class QSerialManager : public QWidget
{
    Q_OBJECT

  public:
    QSerialManager(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_ = nullptr);

  private:
    static constexpr int ARRAY_SIZE = 32;

    static constexpr int BOX_FONT_SIZE = 24;
    static constexpr int BUTTON_FONT_SIZE = 24;
    static constexpr int DISPLAY_FONT_SIZE = 12;

    static constexpr int WAIT_TIME = 1000;
    static constexpr int TIMEOUT = 1;
    static constexpr int BAUD_RATE = 115200;

    static constexpr const char* STATUS_MESSAGE_NAME = "/scorpius/serial_status";
    static constexpr const char* PORTS_SERVICE_NAME = "/scorpius/serial_ports";
    static constexpr const char* CONFIG_SERVICE_NAME = "/scorpius/serial_config";
    static constexpr const char* CONTROLLER_STATE_SERVICE_NAME = "/scorpius/state_controller";
    static constexpr const char* CONTROLLER_STATE_TOPIC_NAME = "/scorpius/state_changed";

    void CB_subStatus(const scorpius_main::msg::SerialStatus& msg_);

    void writeMessage(const QString& message_);

    std::shared_ptr<rclcpp::Node> _node;

    QComboBox* _cbPorts{nullptr};
    QComboBox* _cbStates{nullptr};
    QLabel* _lbState{nullptr};
    QGridLayout* _grid{nullptr};
    QPushButton* _pbConnect{nullptr};
    QPushButton* _pbRefresh{nullptr};
    QTextEdit* _messageDisplay{nullptr};

    AsyncExecutor _executor = AsyncExecutor();

    rclcpp::Client<scorpius_main::srv::SerialPorts>::SharedPtr _client_ports;
    rclcpp::Subscription<scorpius_main::msg::SerialStatus>::SharedPtr _sub_status;
    rclcpp::Publisher<scorpius_main::msg::SerialHeartbeat>::SharedPtr _pub_heartbeat;
    rclcpp::Publisher<scorpius_main::msg::SerialStatus>::SharedPtr _pub_status;
    rclcpp::Subscription<scorpius_main::msg::StateChanged>::SharedPtr _sub_state;
    rclcpp::Client<scorpius_main::srv::SerialConfig>::SharedPtr _client_config;
    rclcpp::Client<scorpius_main::srv::ControllerState>::SharedPtr _client_state;

  signals:
    void serialPortsSignal(const QStringList& port_name_);
    void serialStatusSignal(const scorpius_main::msg::SerialStatus& message_);
    void buttonFinishedSignal(const QString& message_);
    void stateChangedSignal(uint8_t state_);

  private slots:
    void serialPortsSlot(const QStringList& portName_);
    void serialStatusSlot(const scorpius_main::msg::SerialStatus& message_);
    void connectButtonClicked();
    void refreshButtonClicked();
    void changeState(int index_);
    void stateChangedSlot(uint8_t state_);
};

#endif