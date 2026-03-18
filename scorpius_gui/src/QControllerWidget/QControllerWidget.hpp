#ifndef QCONTROLLER_WIDGET_HPP
#define QCONTROLLER_WIDGET_HPP

#include "QControllerProfile.hpp"
#include "scorpius_main/msg/joy.hpp"

#include <rclcpp/rclcpp.hpp>

#include <QWidget>
#include <QPainter>
#include <QSvgRenderer>

#include <array>

class QControllerWidget : public QWidget
{
    Q_OBJECT

  private:
    static constexpr double JOYSTICK_LINE_LENGTH = 0.003;
    static constexpr double FONT_SIZE_RATIO = 0.02;
    static constexpr int MIN_FONT_SIZE = 6;
    static constexpr int MAX_FONT_SIZE = 24;
    static constexpr const char* TOPIC_JOY = "/scorpius/joy";
    static constexpr QColor LIGHT_BLUE{173, 216, 230, 167};

  public:
    QControllerWidget(std::shared_ptr<rclcpp::Node> node_, sControllerProfile profile_, QWidget* parent_ = nullptr);
    ~QControllerWidget();
    void setDeadzone(double deadzone_);
    void setProfile(sControllerProfile profile_);

  protected:
    void paintEvent(QPaintEvent*) override;

  signals:
    void signal_joyMsg(const scorpius_main::msg::Joy& msg_);

  private slots:
    void slot_joyMsg(const scorpius_main::msg::Joy& msg_);

  private:
    void paintRoundButtons(QPainter& p_, QRect& drawRect_);
    void paintPolygonButtons(QPainter& p_, QRect& drawRect_);
    void paintSquareButtons(QPainter& p_, QRect& drawRect_);
    void paintJoystick(QPainter& p_, QRect& drawRect_);

    sControllerProfile _profile;

    double _joyStickDeadzone = 0.0;

    QSvgRenderer _svgRenderer;
    std::shared_ptr<rclcpp::Node> _node;
    rclcpp::Subscription<scorpius_main::msg::Joy>::SharedPtr _sub_joy;

    bool _svgLoaded{false};
};

#endif  // QCONTROLLER_WIDGET_HPP