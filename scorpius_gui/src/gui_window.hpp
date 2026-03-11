#ifndef GUI_WINDOW_HPP
#define GUI_WINDOW_HPP

#include <QMainWindow>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWidget>
#include <QKeyEvent>
#include <rclcpp/rclcpp.hpp>

#include "debug_widget/debug_widget_manager.hpp"
#include "QControllerWidget/QControllerManager.hpp"

class GuiWindow : public QMainWindow
{
    Q_OBJECT
  public:
    explicit GuiWindow(std::shared_ptr<rclcpp::Node> node_, QWidget* parent = nullptr);

    int addTab(QWidget* page, const QString& label);
    bool eventFilter(QObject* obj, QEvent* event);

  private:
    QWidget* _central{nullptr};
    QVBoxLayout* _layout{nullptr};
    QTabWidget* _tabs{nullptr};
    DebugWidgetManager* _debugWidgetManager{nullptr};
    QControllerManager* _controllerManager{nullptr};
};

#endif  // define GUI_WINDOW_HPP