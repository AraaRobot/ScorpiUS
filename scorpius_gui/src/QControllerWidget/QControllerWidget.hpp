#ifndef QCONTROLLER_WIDGET_HPP
#define QCONTROLLER_WIDGET_HPP

#include <QWidget>
#include <QPainter>
#include <QSvgRenderer>
#include <rclcpp/rclcpp.hpp>

class QControllerWidget : public QWidget
{
    Q_OBJECT

  public:
    QControllerWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_ = nullptr);

  protected:
    void paintEvent(QPaintEvent*) override;

  private:
    QSvgRenderer _svgRenderer;
    std::shared_ptr<rclcpp::Node> _node;
};

#endif  // QCONTROLLER_WIDGET_HPP