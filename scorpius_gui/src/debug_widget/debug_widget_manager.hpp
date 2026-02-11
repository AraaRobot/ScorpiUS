#ifndef DEBUG_WIDGET_MANAGER
#define DEBUG_WIDGET_MANAGER

#include <QWidget>
#include <QGridLayout>
#include <array>

#include <rclcpp/rclcpp.hpp>
#include "scorpius_main/msg/teleop.hpp"
#include "debug_widget.hpp"

class DebugWidgetManager : public QWidget
{
    Q_OBJECT

    static constexpr int ROWS = 3;
    static constexpr int COLS = 2;

    static constexpr std::array<char, ROWS* COLS> aNumberToLetter = {'A', 'F', 'B', 'E', 'C', 'D'};

  public:
    DebugWidgetManager(std::shared_ptr<rclcpp::Node> node_, QWidget* parent);

  private:
    QGridLayout* _grid;

    std::shared_ptr<rclcpp::Node> _node;
    rclcpp::Subscription<scorpius_main::msg::Teleop>::SharedPtr _sub_teleop;

    std::array<std::unique_ptr<DebugWidget>, ROWS * COLS> _debugWidgets;
};

#endif  // DEBUG_WIDGET_MANAGER