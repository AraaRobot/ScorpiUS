#include "gui_window.hpp"

GuiWindow::GuiWindow(QWidget* parent_):
    QMainWindow(parent_)
{
    _central = new QWidget(this);
    _layout = new QVBoxLayout(_central);
    _tabs = new QTabWidget(_central);
    _debugWidgetManager = new DebugWidgetManager(_tabs);

    _layout->setContentsMargins(0, 0, 0, 0);
    _layout->setSpacing(0);
    _layout->addWidget(_tabs);

    _central->setLayout(_layout);
    setCentralWidget(_central);
    this->addTab(_debugWidgetManager, "Debug");
}

int GuiWindow::addTab(QWidget* page, const QString& label)
{
  return _tabs->addTab(page, label);
}