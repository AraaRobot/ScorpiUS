#include "debug_widget.hpp"

DebugWidget::DebugWidget(QWidget* parent_, char id_):
    QWidget(parent_)
{
    _ui.setupUi(this);
    this->setPawNumber(id_);

    connect(this, &DebugWidget::setAngleHorizontalSignal, this, &DebugWidget::setAngleHorizontalSlot);
    connect(this, &DebugWidget::setAngleVerticalSignal, this, &DebugWidget::setAngleVerticalSlot);
}

void DebugWidget::setPawNumber(char id_)
{
    _ui.PatteNumber->setText(QStringLiteral("Paw ") + QString(id_));
}

void DebugWidget::setAngleHorizontalSlot(float angle_)
{
    _ui.angle_horizontal->setText(QString::number(angle_, 'f', 1) + QStringLiteral("°"));
}

void DebugWidget::setAngleVerticalSlot(float angle_)
{
    _ui.angle_vertical->setText(QString::number(angle_, 'f', 1) + QStringLiteral("°"));
}