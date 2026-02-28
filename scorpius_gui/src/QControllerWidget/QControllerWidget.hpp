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

class QControllerWidget : public QWidget
{
    Q_OBJECT

  public:
    QControllerWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_ = nullptr);

  protected:
    void paintEvent(QPaintEvent*) override;

  private:
    std::array<Button, 4> buttons = {{{"cross", QPointF(0.7875, 0.413), false},
                                      {"circle", QPointF(0.8547, 0.3128), false},
                                      {"square", QPointF(0.7205, 0.3128), false},
                                      {"triangle", QPointF(0.7875, 0.2139), false}}};

    std::array<DButton, 1> dButtons
        = {{{"up", {{QPointF(0.19, 0.205), QPointF(0.19, 0.26), QPointF(0.215, 0.293), QPointF(0.24, 0.26), QPointF(0.24, 0.205)}}, false}}};

    QSvgRenderer _svgRenderer;
    std::shared_ptr<rclcpp::Node> _node;
};

#endif  // QCONTROLLER_WIDGET_HPP