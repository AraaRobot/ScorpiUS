#ifndef QCONTROLLER_WIDGET_HPP
#define QCONTROLLER_WIDGET_HPP

#include <QWidget>
#include <QPainter>
#include <QSvgRenderer>
#include <rclcpp/rclcpp.hpp>
#include <array>

#include "scorpius_main/msg/joy.hpp"

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
    static constexpr double FONT_SIZE_RATIO = 0.02;
    static constexpr int MIN_FONT_SIZE = 6;
    static constexpr int MAX_FONT_SIZE = 24;
    static constexpr const char* TOPIC_JOY = "/scorpius/joy";
    static constexpr QColor LIGHT_BLUE{173, 216, 230, 167};

  public:
    QControllerWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_ = nullptr);
    ~QControllerWidget();

  protected:
    void paintEvent(QPaintEvent*) override;

  signals:
    void signal_joyMsg(const scorpius_main::msg::Joy& msg_);

  private slots:
    void slot_joyMsg(const scorpius_main::msg::Joy& msg_);

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

    std::array<Button, 6> _backButtons = {{{"L1", QPointF(0.01, 0.1), false},
                                           {"L2", QPointF(0.01, 0.19), false},
                                           {"L3", QPointF(0.329, 0.70), false},
                                           {"R1", QPointF(0.94, 0.1), false},
                                           {"R2", QPointF(0.94, 0.19), false},
                                           {"R3", QPointF(0.624, 0.70), false}}};
    QSvgRenderer _svgRenderer;
    std::shared_ptr<rclcpp::Node> _node;
    rclcpp::Subscription<scorpius_main::msg::Joy>::SharedPtr _sub_joy;
};

#endif  // QCONTROLLER_WIDGET_HPP