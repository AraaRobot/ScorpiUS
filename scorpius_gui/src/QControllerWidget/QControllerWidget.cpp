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

    this->paintButtons(p, drawRect);

    QPen pen1 = p.pen();
    QPen pen2 = pen1;
    pen2.setCapStyle(Qt::RoundCap);
    pen2.setJoinStyle(Qt::RoundJoin);
    p.setPen(pen2);

    this->paintDirectionButtons(p, drawRect);

    p.setPen(pen1);
    p.setBrush(Qt::NoBrush);

    this->paintJoystick(p, drawRect);

    this->paintRLButtons(p, drawRect);
}

void QControllerWidget::paintButtons(QPainter& p_, QRect& drawRect_)
{
    for (const Button& b : _buttons)
    {
        if (!b.pressed && false)
        {
            continue;
        }

        // convert normalized pos to actual coordinates
        int x = drawRect_.left() + b.posNorm.x() * drawRect_.width();
        int y = drawRect_.top() + b.posNorm.y() * drawRect_.height();
        int radius = drawRect_.width() * BUTTON_RADIUS_PERCENT;  // 3% of SVG width

        p_.drawEllipse(QPoint(x, y), radius, radius);
    }
}

void QControllerWidget::paintDirectionButtons(QPainter& p_, QRect& drawRect_)
{
    for (const DButton& d : _dButtons)
    {
        if (!d.pressed && false)
        {
            continue;
        }

        QPolygonF poly;

        for (size_t i = 0; i < 5; ++i)
        {
            int x = drawRect_.left() + d.posNorm[i].x() * drawRect_.width();
            int y = drawRect_.top() + d.posNorm[i].y() * drawRect_.height();
            poly << QPointF(x, y);
        }

        p_.drawPolygon(poly);
    }
}

void QControllerWidget::paintRLButtons(QPainter& p_, QRect& drawRect_)
{
    (void)p_;
    (void)drawRect_;
}
void QControllerWidget::paintJoystick(QPainter& p_, QRect& drawRect_)
{
    
    int radius = drawRect_.width() * JOYSTICK_RADIUS_PERCENT;
    for (const Joystick& j : _joysticks)
    {
        p_.setBrush(QColor(255,255,255,255));
        p_.setPen(Qt::black);
        int x = drawRect_.left() + j.center.x() * drawRect_.width();
        int y = drawRect_.top() + j.center.y() * drawRect_.height();
        p_.drawEllipse(QPointF(x, y), radius, radius);

        p_.setBrush(Qt::NoBrush);
        p_.setPen(Qt::red);
        p_.drawEllipse(QPointF(x, y), radius*JOYSTICK_DEADZONE_PERCENT, radius*JOYSTICK_DEADZONE_PERCENT);
    }
}