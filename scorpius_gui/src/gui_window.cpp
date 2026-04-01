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


    QApplication::instance()->installEventFilter(this);
}

int GuiWindow::addTab(QWidget* page, const QString& label)
{
    return _tabs->addTab(page, label);
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