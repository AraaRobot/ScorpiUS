#ifndef QCONTROLLER_WIDGET_HPP
#define QCONTROLLER_WIDGET_HPP

#include "scorpius_main/msg/joy.hpp"

#include <rclcpp/rclcpp.hpp>

#include <QWidget>
#include <QPainter>
#include <QSvgRenderer>

#include <array>

struct sButton
{
    QString name;     // e.g. "cross", "circle"
    QPointF posNorm;  // normalized x,y (0..1)
    bool pressed;     // state from ROS2
};

struct sDButton
{
    QString name;
    std::array<QPointF, 5> posNorm;
    bool pressed;
};

struct sJoystick
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
    static constexpr double JOYSTICK_LINE_LENGTH = 0.003;
    static constexpr double PS_BUTTON_RADIUS_PERCENT = 0.0252;
    static constexpr double OPTION_BUTTON_RADIUS_PERCENT = 0.01;
    static constexpr double FONT_SIZE_RATIO = 0.02;
    static constexpr int MIN_FONT_SIZE = 6;
    static constexpr int MAX_FONT_SIZE = 24;
    static constexpr const char* TOPIC_JOY = "/scorpius/joy";
    static constexpr QColor LIGHT_BLUE{173, 216, 230, 167};

  public:
    QControllerWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_ = nullptr);
    ~QControllerWidget();
    void setDeadzone(double deadzone_);

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
    void paintPSButton(QPainter& p_, QRect& drawRect_);
    void paintOptionButtons(QPainter& p_, QRect& drawRect_);

    std::array<sButton, 4> _buttons = {{{"cross", QPointF(0.7875, 0.413), false},
                                        {"circle", QPointF(0.8545, 0.3128), false},
                                        {"square", QPointF(0.7205, 0.3128), false},
                                        {"triangle", QPointF(0.7875, 0.2139), false}}};

    std::array<sDButton, 4> _dButtons
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

    std::array<sJoystick, 2> _joysticks
        = {{{"left", QPointF(0.354, .501), 0.0f, 0.0f}, {"right", QPointF(0.649, .501), 0.0f, 0.0f}}};

    std::array<sButton, 6> _backButtons = {{{"L1", QPointF(0.01, 0.1), false},
                                            {"L2", QPointF(0.01, 0.19), false},
                                            {"L3", QPointF(0.329, 0.70), false},
                                            {"R1", QPointF(0.94, 0.1), false},
                                            {"R2", QPointF(0.94, 0.19), false},
                                            {"R3", QPointF(0.624, 0.70), false}}};

    sButton _psButton = {"PS", QPointF(0.5, 0.509), false};

    std::array<sButton, 2> _optionButtons = {{{"Share", QPointF(0.315, 0.1785), false},
                                            {"Options", QPointF(0.6885, 0.1785), false}}};

    double _joyStickDeadzone = 0.0;

    QSvgRenderer _svgRenderer;
    std::shared_ptr<rclcpp::Node> _node;
    rclcpp::Subscription<scorpius_main::msg::Joy>::SharedPtr _sub_joy;
};

#endif  // QCONTROLLER_WIDGET_HPP