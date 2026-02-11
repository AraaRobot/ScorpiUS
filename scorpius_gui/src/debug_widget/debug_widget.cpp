#include "debug_widget.hpp"

DebugWidget::DebugWidget(QWidget* parent_, char id_):
    QWidget(parent_)
{
    _ui.setupUi(this);
    this->setPatteNumber(id_);
}

void DebugWidget::setPatteNumber(char id_)
{
    _ui.PatteNumber->setText(QStringLiteral("Patte ") + QString(id_));
}

void DebugWidget::setAngleHorizontal(float angle_)
{
    _ui.angle_horizontal->setText(QString::number(angle_) + QStringLiteral(" deg"));
}

void DebugWidget::setAngleVertical(float angle_)
{
    _ui.angle_vertical->setText(QString::number(angle_) + QStringLiteral(" deg"));
}