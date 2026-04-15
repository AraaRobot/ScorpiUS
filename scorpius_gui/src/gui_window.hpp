#ifndef GUI_WINDOW_HPP
#define GUI_WINDOW_HPP

#include <QApplication>
#include <QGridLayout>
#include <QKeyEvent>
#include <QMainWindow>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWidget>
#include <rclcpp/rclcpp.hpp>

#include "debug_widget/debug_widget_manager.hpp"
#include "QControllerWidget/QControllerManager.hpp"
#include "QSerialWidget/QSerialManager.hpp"

class GuiWindow : public QMainWindow
{
    Q_OBJECT
  private:
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

  private slots:
    void onCurrentTabChanged(int index);

  private:
    bool eventFilter(QObject* obj, QEvent* event);

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
    QWidget* _dashboard{nullptr};
    QGridLayout* _dashboardLayout{nullptr};
};

#endif  // define GUI_WINDOW_HPP