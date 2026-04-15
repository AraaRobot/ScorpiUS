#include "QControllerWidget.hpp"

QControllerWidget::QControllerWidget(std::shared_ptr<rclcpp::Node> node_, sControllerProfile profile_, QWidget* parent_):
    QWidget(parent_),
    _profile(profile_),
    _node(node_)
{
    _svgLoaded = _svgRenderer.load(_profile.svgPath);
    if (!_svgLoaded)
    {
        RCLCPP_ERROR(_node->get_logger(), "SVG was not loaded");
    }

    setAttribute(Qt::WA_StyledBackground, true);
    setAutoFillBackground(false);  // Critical: stops Qt from filling bg with palette color
    // Single stylesheet, no duplicate
    setStyleSheet("QControllerWidget { background: transparent; }");

    connect(this, &QControllerWidget::signal_joyMsg, this, &QControllerWidget::slot_joyMsg);

    _sub_joy = _node->create_subscription<scorpius_main::msg::Joy>(TOPIC_JOY,
                                                                   10,
                                                                   [this](const scorpius_main::msg::Joy& msg_)
                                                                   {
                                                                       emit this->signal_joyMsg(msg_);
                                                                   });
}

QControllerWidget::~QControllerWidget()
{
    _sub_joy.reset();
}

void QControllerWidget::setProfile(sControllerProfile profile_)
{
    bool ok = _svgRenderer.load(profile_.svgPath);
    if (!ok && !_svgLoaded)
    {
        RCLCPP_ERROR(_node->get_logger(), "SVG was not loaded, no profile to fallback to");
    }
    else if (!ok && _svgLoaded)
    {
        RCLCPP_ERROR(_node->get_logger(), "SVG was not loaded, falling back to old profile");
        _svgRenderer.load(_profile.svgPath);
    }
    else
    {
        _profile = profile_;
        _svgLoaded = true;
        this->update();
    }
}

void QControllerWidget::paintEvent(QPaintEvent*)
{
    if (!_svgLoaded)
    {
        return;
    }

    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.setRenderHint(QPainter::SmoothPixmapTransform);
    p.setPen(QPen(Qt::black, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));

    // draw the SVG to fit the widget while keeping aspect ratio
    QRect targetRect = rect();
    QSize svgSize = _svgRenderer.defaultSize();
    svgSize.scale(targetRect.size(), Qt::KeepAspectRatio);

    QPoint topLeft((this->width() - svgSize.width()) / 2, (this->height() - svgSize.height()) / 2);
    QRect drawRect(topLeft, svgSize);

    _svgRenderer.render(&p, drawRect);

    this->paintRoundButtons(p, drawRect);

    this->paintPolygonButtons(p, drawRect);

    this->paintJoystick(p, drawRect);

    this->paintSquareButtons(p, drawRect);
}

void QControllerWidget::paintRoundButtons(QPainter& p_, QRect& drawRect_)
{
    // 3% of SVG width
    for (const sButtonRound& b : _profile.roundButtons)
    {
        if (b.pressed)
        {
            p_.setBrush(QBrush(LIGHT_BLUE));
        }
        else
        {
            p_.setBrush(Qt::NoBrush);
        }

        // convert normalized pos to actual coordinates
        int x = drawRect_.left() + b.posNorm.x() * drawRect_.width();
        int y = drawRect_.top() + b.posNorm.y() * drawRect_.height();
        int radius = drawRect_.width() * b.radius;

        p_.drawEllipse(QPoint(x, y), radius, radius);
    }
}

void QControllerWidget::paintPolygonButtons(QPainter& p_, QRect& drawRect_)
{
    for (const sButtonPoly& d : _profile.polyButtons)
    {
        if (d.pressed)
        {
            p_.setBrush(QBrush(LIGHT_BLUE));
        }
        else
        {
            p_.setBrush(Qt::NoBrush);
        }

        QPolygonF poly;

        for (size_t i = 0; i < d.posNorm.size(); ++i)
        {
            int x = drawRect_.left() + d.posNorm[i].x() * drawRect_.width();
            int y = drawRect_.top() + d.posNorm[i].y() * drawRect_.height();
            poly << QPointF(x, y);
        }
        p_.drawPolygon(poly);
    }
}

void QControllerWidget::paintSquareButtons(QPainter& p_, QRect& drawRect_)
{
    QFont font;
    int pointSize = std::clamp(int(drawRect_.height() * FONT_SIZE_RATIO), MIN_FONT_SIZE, MAX_FONT_SIZE);
    font.setPointSize(pointSize);
    p_.setFont(font);
    p_.setPen(QPen(Qt::black, 1));
    p_.setBrush(QBrush(LIGHT_BLUE));

    for (const sButtonRect& b : _profile.rectButtons)
    {
        if (b.pressed)
        {
            p_.setBrush(QBrush(LIGHT_BLUE));
        }
        else
        {
            p_.setBrush(Qt::NoBrush);
        }

        // convert normalized pos to actual coordinates
        int x = drawRect_.left() + b.posNorm.x() * drawRect_.width();
        int y = drawRect_.top() + b.posNorm.y() * drawRect_.height();
        int length = drawRect_.width() * b.length;
        int height = drawRect_.width() * b.width;  // 3% of SVG width
        QRect button = QRect(x, y, length, height);
        p_.drawRect(button);
        p_.drawText(button, Qt::AlignCenter, QString(b.name));
    }
}
void QControllerWidget::paintJoystick(QPainter& p_, QRect& drawRect_)
{
    for (const sJoystick& j : _profile.joysticks)
    {
        int radius = drawRect_.width() * j.radius;
        p_.setBrush(Qt::white);
        p_.setPen(Qt::black);
        float x = drawRect_.left() + j.center.x() * drawRect_.width();
        float y = drawRect_.top() + j.center.y() * drawRect_.height();
        p_.drawEllipse(QPointF(x, y), radius, radius);

        p_.setBrush(Qt::NoBrush);
        p_.setPen(Qt::red);
        p_.drawEllipse(QPointF(x, y), radius * _joyStickDeadzone, radius * _joyStickDeadzone);

        p_.setPen(QPen(Qt::darkGreen, 2));
        float line1x1 = x + j.xPos * radius + drawRect_.width() * JOYSTICK_LINE_LENGTH;
        float line1x2 = x + j.xPos * radius - drawRect_.width() * JOYSTICK_LINE_LENGTH;
        float line1y1 = y - j.yPos * radius;
        float line1y2 = y - j.yPos * radius;
        QLine Line1 = QLine(line1x1, line1y1, line1x2, line1y2);

        float line2x1 = x + j.xPos * radius;
        float line2x2 = x + j.xPos * radius;
        float line2y1 = y - j.yPos * radius + drawRect_.width() * JOYSTICK_LINE_LENGTH;
        float line2y2 = y - j.yPos * radius - drawRect_.width() * JOYSTICK_LINE_LENGTH;
        QLine Line2 = QLine(line2x1, line2y1, line2x2, line2y2);

        p_.drawLine(Line1);
        p_.drawLine(Line2);
    }
}

void QControllerWidget::slot_joyMsg(const scorpius_main::msg::Joy& msg_)
{
    for (sButtonRound& b : _profile.roundButtons)
    {
        if (b.joyIndex < 0 || b.joyIndex >= scorpius_main::msg::Joy::MAX)
        {
            continue;
        }
        b.pressed = static_cast<bool>(msg_.joy_data[b.joyIndex]);
    }

    for (sButtonPoly& b : _profile.polyButtons)
    {
        if (b.joyIndex < 0 || b.joyIndex >= scorpius_main::msg::Joy::MAX)
        {
            continue;
        }
        b.pressed = static_cast<bool>(msg_.joy_data[b.joyIndex]);
    }

    for (sButtonRect& b : _profile.rectButtons)
    {
        if (b.joyIndex < 0 || b.joyIndex >= scorpius_main::msg::Joy::MAX)
        {
            continue;
        }
        b.pressed = static_cast<bool>(msg_.joy_data[b.joyIndex]);
    }

    for (sJoystick& j : _profile.joysticks)
    {
        if (j.joyIndexHoriz < 0 || j.joyIndexHoriz >= scorpius_main::msg::Joy::MAX)
        {
            continue;
        }
        if (j.joyIndexVert < 0 || j.joyIndexVert >= scorpius_main::msg::Joy::MAX)
        {
            continue;
        }
        j.xPos = std::clamp<float>(msg_.joy_data[j.joyIndexHoriz], -1.0f, 1.0f);
        j.yPos = std::clamp<float>(msg_.joy_data[j.joyIndexVert], -1.0f, 1.0f);
    }

    this->update();
}

void QControllerWidget::setDeadzone(double deadzone_)
{
    _joyStickDeadzone = deadzone_;
    this->update();
}