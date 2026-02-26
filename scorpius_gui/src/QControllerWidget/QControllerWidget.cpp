#include "QControllerWidget.hpp"

QControllerWidget::QControllerWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_),
    _node(node_)
{
    bool loadOk = _svgRenderer.load(QString(":/images/images/Dualshock_4_Layout.svg"));
    if (!loadOk)
    {
        RCLCPP_ERROR(_node->get_logger(), "SVG was not loaded");
    }
}

void QControllerWidget::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.setRenderHint(QPainter::SmoothPixmapTransform);

    // draw the SVG to fit the widget while keeping aspect ratio
    QRect targetRect = rect();
    QSize svgSize = _svgRenderer.defaultSize();
    svgSize.scale(targetRect.size(), Qt::KeepAspectRatio);

    QPoint topLeft((width() - svgSize.width()) / 2, (height() - svgSize.height()) / 2);
    QRect drawRect(topLeft, svgSize);

    _svgRenderer.render(&p, drawRect);

    p.setBrush(QBrush(QColor(173, 216, 230, 167)));
    for (const Button& b : buttons)
    {
        if (!b.pressed && false)
        {
            continue;
        }

        // convert normalized pos to actual coordinates
        int x = drawRect.left() + b.posNorm.x() * drawRect.width();
        int y = drawRect.top() + b.posNorm.y() * drawRect.height();
        int radius = drawRect.width() * 0.03;  // 3% of SVG width

        p.drawEllipse(QPoint(x, y), radius, radius);
    }
}