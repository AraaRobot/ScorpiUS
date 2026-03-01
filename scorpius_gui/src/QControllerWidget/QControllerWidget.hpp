#ifndef QCONTROLLER_WIDGET_HPP
#define QCONTROLLER_WIDGET_HPP

#include <QWidget>
#include <QPainter>
#include <QSvgRenderer>
#include <rclcpp/rclcpp.hpp>
#include <array>

struct Button
{
    QString name;     // e.g. "cross", "circle"
    QPointF posNorm;  // normalized x,y (0..1)
    bool pressed;     // state from ROS2
};

struct DButton
{
    QString name;
    std::array<QPointF, 5> posNorm;
    bool pressed;
};

struct Joystick
{
    QString name;
    QPointF center;
    float xPos;
    float yPos;
};

class QControllerWidget : public QWidget
{
    Q_OBJECT

  private:
    static constexpr double BUTTON_RADIUS_PERCENT = 0.03;  // in percents of total image width
    static constexpr double JOYSTICK_RADIUS_PERCENT = 0.06;
    static constexpr double JOYSTICK_DEADZONE_PERCENT = 0.25;

  public:
    QControllerWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_ = nullptr);

  protected:
    void paintEvent(QPaintEvent*) override;

  private:
    void paintButtons(QPainter& p_, QRect& drawRect_);
    void paintDirectionButtons(QPainter& p_, QRect& drawRect_);
    void paintRLButtons(QPainter& p_, QRect& drawRect_);
    void paintJoystick(QPainter& p_, QRect& drawRect_);

    std::array<Button, 4> _buttons = {{{"cross", QPointF(0.7875, 0.413), false},
                                       {"circle", QPointF(0.8537, 0.3128), false},
                                       {"square", QPointF(0.7205, 0.3128), false},
                                       {"triangle", QPointF(0.7875, 0.2139), false}}};

    std::array<DButton, 4> _dButtons
        = {{{"up",
             {{QPointF(0.19, 0.205), QPointF(0.19, 0.26), QPointF(0.215, 0.293), QPointF(0.24, 0.26), QPointF(0.24, 0.205)}},
             false},
            {"down",
             {{QPointF(0.19, 0.42), QPointF(0.19, 0.365), QPointF(0.215, 0.335), QPointF(0.24, 0.365), QPointF(0.24, 0.42)}},
             false},
            {"left",
             {{QPointF(0.14, 0.279), QPointF(0.18, 0.279), QPointF(0.20, 0.31), QPointF(0.18, 0.347), QPointF(0.14, 0.347)}},
             false},
            {"right",
             {{QPointF(0.287, 0.279), QPointF(0.247, 0.279), QPointF(0.227, 0.31), QPointF(0.247, 0.347), QPointF(0.287, 0.347)}},
             false}}};

    std::array<Joystick, 2> _joysticks
        = {{{"left", QPointF(0.354, .501), 0.0f, 0.0f}, {"right", QPointF(0.649, .501), 0.0f, 0.0f}}};

    std::array<Button, 4> _backButtons = {{{"L1", QPointF(0.01, 0.1), false},
                                           {"L2", QPointF(0.01, 0.19), false},
                                           {"R1", QPointF(0.94, 0.1), false},
                                           {"R2", QPointF(0.94, 0.19), false}}};
    QSvgRenderer _svgRenderer;
    std::shared_ptr<rclcpp::Node> _node;
};

#endif  // QCONTROLLER_WIDGET_HPP