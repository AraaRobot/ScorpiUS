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

void QControllerWidget::paintEvent(QPaintEvent*)
{
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

    this->paintButtons(p, drawRect);

    this->paintDirectionButtons(p, drawRect);

    p.setBrush(Qt::NoBrush);

    this->paintJoystick(p, drawRect);

    this->paintRLButtons(p, drawRect);
}

void QControllerWidget::paintButtons(QPainter& p_, QRect& drawRect_)
{
    for (const sButton& b : _buttons)
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
        int radius = drawRect_.width() * BUTTON_RADIUS_PERCENT;  // 3% of SVG width

        p_.drawEllipse(QPoint(x, y), radius, radius);
    }
}

void QControllerWidget::paintDirectionButtons(QPainter& p_, QRect& drawRect_)
{
    for (const sDButton& d : _dButtons)
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
    QFont font;
    int pointSize = std::clamp(int(drawRect_.height() * FONT_SIZE_RATIO), MIN_FONT_SIZE, MAX_FONT_SIZE);
    font.setPointSize(pointSize);
    p_.setFont(font);
    p_.setPen(QPen(Qt::black, 1));
    p_.setBrush(QBrush(QColor(173, 216, 230, 167)));

    for (const sButton& b : _backButtons)
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
        int length = drawRect_.width() * 0.05;
        int height = drawRect_.width() * 0.03;  // 3% of SVG width
        QRect button = QRect(x, y, length, height);
        p_.drawRect(button);
        p_.drawText(button, Qt::AlignCenter, QString(b.name));
    }
}
void QControllerWidget::paintJoystick(QPainter& p_, QRect& drawRect_)
{
    int radius = drawRect_.width() * JOYSTICK_RADIUS_PERCENT;
    for (const sJoystick& j : _joysticks)
    {
        p_.setBrush(Qt::white);
        p_.setPen(Qt::black);
        int x = drawRect_.left() + j.center.x() * drawRect_.width();
        int y = drawRect_.top() + j.center.y() * drawRect_.height();
        p_.drawEllipse(QPointF(x, y), radius, radius);

        p_.setBrush(Qt::NoBrush);
        p_.setPen(Qt::red);
        p_.drawEllipse(QPointF(x, y), radius * _joyStickDeadzone, radius * _joyStickDeadzone);

        p_.setPen(QPen(Qt::darkGreen, 2));
        float line1x1 = x + j.xPos * radius + drawRect_.width() * 0.003;
        float line1x2 = x + j.xPos * radius - drawRect_.width() * 0.003;
        float line1y1 = y + j.yPos * radius;
        float line1y2 = y + j.yPos * radius;
        QLine Line1 = QLine(line1x1, line1y1, line1x2, line1y2);

        float line2x1 = x + j.xPos * radius;
        float line2x2 = x + j.xPos * radius;
        float line2y1 = y + j.yPos * radius + drawRect_.width() * 0.003;
        float line2y2 = y + j.yPos * radius - drawRect_.width() * 0.003;
        QLine Line2 = QLine(line2x1, line2y1, line2x2, line2y2);

        p_.drawLine(Line1);
        p_.drawLine(Line2);
    }
}

void QControllerWidget::slot_joyMsg(const scorpius_main::msg::Joy& msg_)
{
    _buttons[0].pressed = static_cast<bool>(msg_.joy_data[scorpius_main::msg::Joy::A]);
    _buttons[1].pressed = static_cast<bool>(msg_.joy_data[scorpius_main::msg::Joy::B]);
    _buttons[2].pressed = static_cast<bool>(msg_.joy_data[scorpius_main::msg::Joy::X]);
    _buttons[3].pressed = static_cast<bool>(msg_.joy_data[scorpius_main::msg::Joy::Y]);

    _dButtons[0].pressed = static_cast<bool>(msg_.joy_data[scorpius_main::msg::Joy::CROSS_UP]);
    _dButtons[1].pressed = static_cast<bool>(msg_.joy_data[scorpius_main::msg::Joy::CROSS_DOWN]);
    _dButtons[2].pressed = static_cast<bool>(msg_.joy_data[scorpius_main::msg::Joy::CROSS_LEFT]);
    _dButtons[3].pressed = static_cast<bool>(msg_.joy_data[scorpius_main::msg::Joy::CROSS_RIGHT]);

    _joysticks[0].xPos = std::clamp<float>(msg_.joy_data[scorpius_main::msg::Joy::JOYSTICK_LEFT_HORIZ], -1.0f, 1.0f);
    _joysticks[0].yPos = std::clamp<float>(msg_.joy_data[scorpius_main::msg::Joy::JOYSTICK_LEFT_VERT], -1.0f, 1.0f);

    _joysticks[1].xPos = std::clamp<float>(msg_.joy_data[scorpius_main::msg::Joy::JOYSTICK_RIGHT_HORIZ], -1.0f, 1.0f);
    _joysticks[1].yPos = std::clamp<float>(msg_.joy_data[scorpius_main::msg::Joy::JOYSTICK_RIGHT_VERT], -1.0f, 1.0f);

    _backButtons[0].pressed = static_cast<bool>(msg_.joy_data[scorpius_main::msg::Joy::L1]);
    _backButtons[1].pressed = static_cast<bool>(msg_.joy_data[scorpius_main::msg::Joy::L2]);
    _backButtons[2].pressed = static_cast<bool>(msg_.joy_data[scorpius_main::msg::Joy::L3]);
    _backButtons[3].pressed = static_cast<bool>(msg_.joy_data[scorpius_main::msg::Joy::R1]);
    _backButtons[4].pressed = static_cast<bool>(msg_.joy_data[scorpius_main::msg::Joy::R2]);
    _backButtons[5].pressed = static_cast<bool>(msg_.joy_data[scorpius_main::msg::Joy::R3]);

    this->update();
}

void QControllerWidget::setDeadzone(double deadzone_)
{
    _joyStickDeadzone = deadzone_;
    this->update();
}