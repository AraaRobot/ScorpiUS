#include "gui_window.hpp"

#include <rclcpp/qos.hpp>
#include <rclcpp/duration.hpp>

GuiWindow::GuiWindow(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QMainWindow(parent_),
    _node(node_)
{
    _central = new QWidget(this);
    _layout = new QVBoxLayout(_central);
    _tabs = new QTabWidget(_central);

    _debugWidgetManager = new DebugWidgetManager(node_, _central);
    _controllerManager = new QControllerManager(node_, _central);
    _controllerManager->update();
    _serialManager = new QSerialManager(node_, _central);

    _debugTab = new QWidget(_tabs);
    _debugTab->setObjectName("debugTab");
    _debugTab->setAttribute(Qt::WA_StyledBackground, true);
    _debugTabLayout = new QVBoxLayout(_debugTab);

    _controllerTab = new QWidget(_tabs);
    _controllerTab->setObjectName("controllerTab");
    _controllerTab->setAttribute(Qt::WA_StyledBackground, true);
    _controllerTabLayout = new QVBoxLayout(_controllerTab);

    _serialTab = new QWidget(_tabs);
    _serialTab->setObjectName("serialTab");
    _serialTab->setAttribute(Qt::WA_StyledBackground, true);
    _serialTabLayout = new QVBoxLayout(_serialTab);

    _dashboard = new QWidget(_tabs);
    _dashboard->setObjectName("dashboardTab");
    _dashboard->setAttribute(Qt::WA_StyledBackground, true);
    _dashboardLayout = new QGridLayout(_dashboard);

    _debugTabLayout->setContentsMargins(0, 0, 0, 0);
    _debugTabLayout->setSpacing(0);
    _debugTab->setLayout(_debugTabLayout);

    _controllerTabLayout->setContentsMargins(0, 0, 0, 0);
    _controllerTabLayout->setSpacing(0);
    _controllerTab->setLayout(_controllerTabLayout);

    _serialTabLayout->setContentsMargins(0, 0, 0, 0);
    _serialTabLayout->setSpacing(0);
    _serialTab->setLayout(_serialTabLayout);

    _dashboardLayout->setContentsMargins(0, 0, 0, 0);
    _dashboardLayout->setSpacing(0);
    _dashboard->setLayout(_dashboardLayout);
    _blinker = new HeartbeatBlinker(this);
    _blinker->setFixedSize(12, 12);
    connect(this, &GuiWindow::toggleBlink, _blinker, &HeartbeatBlinker::toggleBlinkImplementation, Qt::QueuedConnection);
    _blinker->update();

    _layout->setContentsMargins(0, 0, 0, 0);
    _layout->setSpacing(0);
    _layout->addWidget(_blinker);
    _layout->addWidget(_tabs);

    _central->setLayout(_layout);
    setCentralWidget(_central);
    this->addTab(_debugTab, "Debug");
    this->addTab(_controllerTab, "Controller");
    this->addTab(_serialTab, "Serial");

    this->setBackground();
    this->addTab(_dashboard, "Dashboard");

    connect(_tabs, &QTabWidget::currentChanged, this, &GuiWindow::onCurrentTabChanged);
    onCurrentTabChanged(_tabs->currentIndex());

    QApplication::instance()->installEventFilter(this);
    this->setupSubHeartbeat();
}

int GuiWindow::addTab(QWidget* page, const QString& label)
{
    return _tabs->addTab(page, label);
}

void GuiWindow::setBackground()
{
    _tabs->setStyleSheet("QTabWidget::pane { background: transparent; }"
                         "QTabBar::tab { background: transparent; }");

    QString _bgImageStyle = "border-image: url(:/images/images/gui_wallpaper_vert.png) 0 0 0 0 round round;";
    if (std::rand() % 10 == 1)
    {
        _bgImageStyle = "border-image: url(:/images/images/gui_wallpaper_vert_easter_egg.png) 0 0 0 0 round round;";
    }
    _debugWidgetManager->setObjectName("debugWidgetManager");
    _controllerManager->setObjectName("controllerManager");
    _serialManager->setObjectName("serialManager");

    _debugTab->setStyleSheet("#debugTab {" + _bgImageStyle + "}"
                             "#debugWidgetManager, #debugWidgetManager * {"
                             "    background-color: rgba(255, 255, 255, 0.8);"
                             "    color: black;"
                             "    border: 1px solid rgba(0, 0, 0, 0.12);"
                             "}");
    _controllerTab->setStyleSheet("#controllerTab {" + _bgImageStyle + "}"
                                  "#controllerManager {"
                                  "    background: transparent;"
                                  "}"
                                  "#controllerManager * {"
                                  "    color: black;"
                                  "}");
    _serialTab->setStyleSheet("#serialTab {" + _bgImageStyle + "}"
                               "#serialManager, #serialManager * {"
                               "    background-color: rgba(255, 255, 255, 0.8);"
                               "    color: black;"
                               "    border: 1px solid rgba(0, 0, 0, 0.12);"
                               "}");
    _dashboard->setStyleSheet("#dashboardTab {" + _bgImageStyle + "}"
                              "#debugWidgetManager, #debugWidgetManager * {"
                              "    background-color: rgba(255, 255, 255, 0.8);"
                              "    color: black;"
                              "    border: 1px solid rgba(0, 0, 0, 0.12);"
                              "}"
                              "#controllerManager {"
                              "    background: transparent;"
                              "}"
                              "#controllerManager * {"
                              "    color: black;"
                              "}"
                              "#serialManager, #serialManager * {"
                              "    background-color: rgba(255, 255, 255, 0.8);"
                              "    color: black;"
                              "    border: 1px solid rgba(0, 0, 0, 0.12);"
                              "}");
}

void GuiWindow::onCurrentTabChanged(int index)
{
    _debugWidgetManager->hide();
    _controllerManager->hide();
    _serialManager->hide();

    _debugTabLayout->removeWidget(_debugWidgetManager);
    _controllerTabLayout->removeWidget(_controllerManager);
    _serialTabLayout->removeWidget(_serialManager);
    _dashboardLayout->removeWidget(_debugWidgetManager);
    _dashboardLayout->removeWidget(_controllerManager);
    _dashboardLayout->removeWidget(_serialManager);

    switch (index)
    {
        case std::to_underlying(eTabs::DEBUG):
            _debugTabLayout->addWidget(_debugWidgetManager);
            _debugWidgetManager->show();
            break;
        case std::to_underlying(eTabs::CONTROLLER):
            _controllerTabLayout->addWidget(_controllerManager);
            _controllerManager->show();
            break;
        case std::to_underlying(eTabs::SERIAL):
            _serialTabLayout->addWidget(_serialManager);
            _serialManager->show();
            break;
        case std::to_underlying(eTabs::DASHBOARD):
            _dashboardLayout->addWidget(_debugWidgetManager, 0, 0);
            _dashboardLayout->addWidget(_controllerManager, 0, 1);
            _dashboardLayout->addWidget(_serialManager, 1, 0, 1, 2);
            _debugWidgetManager->show();
            _controllerManager->show();
            _serialManager->show();
            break;
        default:
            break;
    }
}

bool GuiWindow::eventFilter(QObject* obj, QEvent* event)
{
    if (event->type() == QEvent::KeyPress)
    {
        QKeyEvent* keyEvent = static_cast<QKeyEvent*>(event);

        if (keyEvent->modifiers() == Qt::NoModifier)
        {
            switch (keyEvent->key())
            {
                case Qt::Key_D:
                    _tabs->setCurrentIndex(0);
                    return true;
                case Qt::Key_C:
                    _tabs->setCurrentIndex(1);
                    return true;
                case Qt::Key_S:
                    _tabs->setCurrentIndex(2);
                    return true;
                case Qt::Key_Space:
                    this->setBackground();
                    return true;
                default:
                    break;
            }
        }
    }
    return QMainWindow::eventFilter(obj, event);
}

void GuiWindow::setupSubHeartbeat()
{
    rclcpp::QoS heartbeatQoS = rclcpp::QoS(1)
                                   .history(rclcpp::HistoryPolicy::KeepLast)
                                   .reliability(rclcpp::ReliabilityPolicy::Reliable)
                                   .durability(rclcpp::DurabilityPolicy::Volatile)
                                   .deadline(rclcpp::Duration::from_seconds(HEARTBEAT_DEADLINE_S))
                                   .lifespan(rclcpp::Duration::from_seconds(HEARTBEAT_LIFESPAN))
                                   .liveliness(rclcpp::LivelinessPolicy::Automatic)
                                   .liveliness_lease_duration(rclcpp::Duration::from_seconds(HEARTBEAT_LEASE_DURATION));

    rclcpp::SubscriptionOptions options = rclcpp::SubscriptionOptions();

    options.event_callbacks.deadline_callback = [this](rclcpp::QOSDeadlineRequestedInfo& /*info_*/)
    {
        emit toggleBlink(HeartbeatBlinker::eState::DISCONNECTED);
    };

    options.event_callbacks.liveliness_callback = [this](rclcpp::QOSLivelinessChangedInfo& info_)
    {
        if (info_.alive_count == 0)
        {
            RCLCPP_WARN(_node->get_logger(), "Publisher lost liveliness");
            emit toggleBlink(HeartbeatBlinker::eState::DISCONNECTED);
        }
    };

    options.event_callbacks.incompatible_qos_callback = [this](rclcpp::QOSRequestedIncompatibleQoSInfo& event_)
    {
        RCLCPP_FATAL(_node->get_logger(), "Incompatible QoS detected! Last policy kind: %d", event_.last_policy_kind);
    };

    options.event_callbacks.message_lost_callback = [this](rclcpp::QOSMessageLostInfo& info_)
    {
        RCLCPP_ERROR(_node->get_logger(),
                     "Heartbeat messages lost! total=%ld delta=%ld",
                     info_.total_count,
                     info_.total_count_change);
    };

    _sub_heartbeat = _node->create_subscription<scorpius_main::msg::SerialHeartbeat>(
        HEARTBEAT_TOPIC_NAME,
        heartbeatQoS,
        [this](const scorpius_main::msg::SerialHeartbeat& msg_)
        {
            emit this->toggleBlink(msg_.alive ? HeartbeatBlinker::eState::ALIVE : HeartbeatBlinker::eState::DEAD);
        },
        options);

    if (_sub_heartbeat->get_publisher_count() == 0)
    {
        RCLCPP_WARN(_node->get_logger(), "No publishers on heartbeat topic");
    }
}