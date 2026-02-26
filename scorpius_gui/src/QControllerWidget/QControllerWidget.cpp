#include "QControllerWidget.hpp"

QControllerWidget::QControllerWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_), _node(node_)
{
    bool loadOk = _svgRenderer.load(QString(":/images/images/Dualshock_4_Layout.svg"));
    if(!loadOk)
    {
        RCLCPP_ERROR(_node->get_logger(), "SVG was not loaded");
    }
}

void QControllerWidget::paintEvent(QPaintEvent*)
{
    if (_svgRenderer.isValid())
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
    }
}