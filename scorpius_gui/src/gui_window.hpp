#ifndef GUI_WINDOW_HPP
#define GUI_WINDOW_HPP

#include "debug_widget/debug_widget_manager.hpp"
#include "QControllerWidget/QControllerManager.hpp"
#include "QSerialWidget/QSerialManager.hpp"
#include "QHeartbeat/QHeartbeatBlinker.hpp"
#include "scorpius_main/msg/serial_heartbeat.hpp"

#include <QApplication>
#include <QGridLayout>
#include <QKeyEvent>
#include <QMainWindow>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWidget>
#include <rclcpp/rclcpp.hpp>

#include <utility>


class GuiWindow : public QMainWindow
{
    Q_OBJECT

  private:
    static constexpr const char* HEARTBEAT_TOPIC_NAME = "/scorpius/serial_heartbeat";
    static constexpr double HEARTBEAT_DEADLINE_S = 1.0;
    static constexpr double HEARTBEAT_LIFESPAN = 1.5;
    static constexpr double HEARTBEAT_LEASE_DURATION = 1.5;

    enum class eTabs : int
    {
        DEBUG,
        CONTROLLER,
        SERIAL,
        DASHBOARD
    };

  public:
    explicit GuiWindow(std::shared_ptr<rclcpp::Node> node_, QWidget* parent = nullptr);

    int addTab(QWidget* page, const QString& label);

  signals:
    void toggleBlink(HeartbeatBlinker::eState state_);

  private slots:
    void onCurrentTabChanged(int index);

  private:
    bool eventFilter(QObject* obj, QEvent* event);
    void setupSubHeartBeat();

    QWidget* _central{nullptr};
    QVBoxLayout* _layout{nullptr};
    QTabWidget* _tabs{nullptr};
    QWidget* _debugTab{nullptr};
    QVBoxLayout* _debugTabLayout{nullptr};
    QWidget* _controllerTab{nullptr};
    QVBoxLayout* _controllerTabLayout{nullptr};
    QWidget* _serialTab{nullptr};
    QVBoxLayout* _serialTabLayout{nullptr};
    DebugWidgetManager* _debugWidgetManager{nullptr};
    QControllerManager* _controllerManager{nullptr};
    QSerialManager* _serialManager{nullptr};
    HeartbeatBlinker* _blinker{nullptr};
    QWidget* _dashboard{nullptr};
    QGridLayout* _dashboardLayout{nullptr};

    rclcpp::Subscription<scorpius_main::msg::SerialHeartbeat>::SharedPtr _sub_heartbeat{nullptr};
    rclcpp::Node::SharedPtr _node;
    bool _blinkAlive = false;
};

#endif  // define GUI_WINDOW_HPP