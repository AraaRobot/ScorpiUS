#ifndef DEBUG_WIDGET_MANAGER
#define DEBUG_WIDGET_MANAGER

#include <QWidget>
#include <QGridLayout>
#include <array>

#include <rclcpp/rclcpp.hpp>
#include "scorpius_main/msg/servo_angles.hpp"
#include "debug_widget.hpp"

class DebugWidgetManager : public QWidget
{
    Q_OBJECT

    static constexpr int ROWS = 3;
    static constexpr int COLS = 2;

    static constexpr std::array<char, ROWS* COLS> aNumberToLetter = {'A', 'F', 'B', 'E', 'C', 'D'};

    static int letterToIndex(char letter)
    {
        if (letter < 'A' || letter > 'Z')
        {
            return -1;
        }
        for (int i = 0; i < ROWS * COLS; ++i)
        {
            if (aNumberToLetter[i] == letter)
            {
                return i;
            }
        }
        return -1;
    }
    static char indexToLetter(int idx)
    {
        return (idx >= 0 && idx < ROWS * COLS) ? aNumberToLetter[idx] : '?';
    }

  public:
    DebugWidgetManager(std::shared_ptr<rclcpp::Node> node_, QWidget* parent);

  private:
    void CB_subTeleop(const scorpius_main::msg::ServoAngles& msg_);

    QGridLayout* _grid;

    std::shared_ptr<rclcpp::Node> _node;
    rclcpp::Subscription<scorpius_main::msg::ServoAngles>::SharedPtr _sub_teleop;

    std::array<std::unique_ptr<DebugWidget>, ROWS * COLS> _debugWidgets;
};

#endif  // DEBUG_WIDGET_MANAGER