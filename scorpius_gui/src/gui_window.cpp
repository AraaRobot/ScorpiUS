#include "gui_window.hpp"

GuiWindow::GuiWindow(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QMainWindow(parent_)
{
    _central = new QWidget(this);
    _layout = new QVBoxLayout(_central);
    _tabs = new QTabWidget(_central);
    _debugWidgetManager = new DebugWidgetManager(node_, _tabs);
    _controllerManager = new QControllerManager(node_, _tabs);
    _controllerManager->update();
    _serialManager = new QSerialManager(node_, _tabs);

    _layout->setContentsMargins(0, 0, 0, 0);
    _layout->setSpacing(0);
    _layout->addWidget(_tabs);

    _central->setLayout(_layout);
    setCentralWidget(_central);
    this->addTab(_debugWidgetManager, "Debug");
    this->addTab(_controllerManager, "Controller");
    this->addTab(_serialManager, "Serial");

    this->setBackground();

    QApplication::instance()->installEventFilter(this);
}

int GuiWindow::addTab(QWidget* page, const QString& label)
{
    return _tabs->addTab(page, label);
}

void GuiWindow::setBackground()
{
    _tabs->setStyleSheet(
        "QTabWidget::pane { background: transparent; }"
        "QTabBar::tab { background: transparent; }"
    );

    QString _bgImageStyle = "border-image: url(:/images/images/gui_wallpaper_vert.png) 0 0 0 0 round round;";
    if (std::rand() % 10 == 1)
    {
        _bgImageStyle = "border-image: url(:/images/images/gui_wallpaper_vert_easter_egg.png) 0 0 0 0 round round;";
    }
    _debugWidgetManager->setObjectName("debugWidgetManager");
    _debugWidgetManager->setAttribute(Qt::WA_StyledBackground, true);
    _controllerManager->setObjectName("controllerManager");
    _controllerManager->setAttribute(Qt::WA_StyledBackground, true);
    _serialManager->setObjectName("serialManager");
    _serialManager->setAttribute(Qt::WA_StyledBackground, true);

    _debugWidgetManager->setStyleSheet(
        "#debugWidgetManager {"
        + _bgImageStyle +
        "}"
        "#debugWidgetManager * {"
        "    background: rgba(255, 255, 255, 0.7);"
        "}"
    );
    _controllerManager->setStyleSheet(
        "#controllerManager {"
        + _bgImageStyle +
        "}"
        "#controllerManager * {"
        "    background: rgba(255, 255, 255, 0.7);"
        "}"
    );
    _serialManager->setStyleSheet(
        "#serialManager {"
        + _bgImageStyle +
        "}"
        "#serialManager * {"
        "    background: rgba(255, 255, 255, 0.7);"
        "}"
    );
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