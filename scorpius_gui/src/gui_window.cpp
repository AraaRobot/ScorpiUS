#include "gui_window.hpp"

GuiWindow::GuiWindow(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QMainWindow(parent_)
{
    _central = new QWidget(this);
    _layout = new QVBoxLayout(_central);
    _tabs = new QTabWidget(_central);

    _debugTab = new QWidget(_tabs);
    _debugTabLayout = new QVBoxLayout(_debugTab);
    _controllerTab = new QWidget(_tabs);
    _controllerTabLayout = new QVBoxLayout(_controllerTab);
    _serialTab = new QWidget(_tabs);
    _serialTabLayout = new QVBoxLayout(_serialTab);

    _debugWidgetManager = new DebugWidgetManager(node_, _central);
    _controllerManager = new QControllerManager(node_, _central);
    _controllerManager->update();
    _serialManager = new QSerialManager(node_, _central);

    _dashboard = new QWidget(_tabs);
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

    _layout->setContentsMargins(0, 0, 0, 0);
    _layout->setSpacing(0);
    _layout->addWidget(_tabs);

    _central->setLayout(_layout);
    setCentralWidget(_central);
    this->addTab(_debugTab, "Debug");
    this->addTab(_controllerTab, "Controller");
    this->addTab(_serialTab, "Serial");
    this->addTab(_dashboard, "Dashboard");

    connect(_tabs, &QTabWidget::currentChanged, this, &GuiWindow::onCurrentTabChanged);
    onCurrentTabChanged(_tabs->currentIndex());

    QApplication::instance()->installEventFilter(this);
}

int GuiWindow::addTab(QWidget* page, const QString& label)
{
    return _tabs->addTab(page, label);
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
                default:
                    break;
            }
        }
    }
    return QMainWindow::eventFilter(obj, event);
}