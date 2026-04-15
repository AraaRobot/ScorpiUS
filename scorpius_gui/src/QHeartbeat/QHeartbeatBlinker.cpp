#include "QHeartbeatBlinker.hpp"

#include <QPainter>

HeartbeatBlinker::HeartbeatBlinker(QWidget* parent_):
    QWidget(parent_)
{
    connect(&_timer, &QTimer::timeout, this, &HeartbeatBlinker::toggleBlinkStep);
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
}

void HeartbeatBlinker::setPeriod(int period_)
{
    _blinkPeriod = period_;
    if (_timer.isActive())
    {
        _timer.setInterval(period_);
    }
}

void HeartbeatBlinker::toggleBlinkImplementation(eState state_)
{
    if (_blinkState == state_)
    {
        return;
    }

    _blinkState = state_;
    if (state_ == eState::ALIVE)
    {
        _timer.start(_blinkPeriod);
    }
    else if (state_ == eState::DEAD)
    {
        _timer.stop();
        _blinkStep = false;
    }
    else if (state_ == eState::DISCONNECTED)
    {
        _timer.stop();
        _blinkStep = false;
    }
    this->update();
}

void HeartbeatBlinker::toggleBlinkStep()
{
    _blinkStep = !_blinkStep;
    this->update();
}

void HeartbeatBlinker::paintEvent(QPaintEvent* /*event*/)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    const int side = qMin(width(), height());
    const QRect square((width() - side) / 2, (height() - side) / 2, side, side);
    if (_blinkState == eState::ALIVE)
    {
        if (_blinkStep)
        {
            p.setBrush(Qt::green);
        }
        else
        {
            p.setBrush(Qt::NoBrush);
        }
    }
    else if (_blinkState == eState::DEAD)
    {
        p.setBrush(Qt::red);
    }
    else if (_blinkState == eState::DISCONNECTED)
    {
        p.setBrush(Qt::yellow);
    }

    p.drawRect(square);
}