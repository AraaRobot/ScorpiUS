#include "debug_widget.hpp"

DebugWidget::DebugWidget(QWidget* parent_, size_t number_) : QWidget(parent_)
{
    _ui.setupUi(this);
    this->setPatteNumber(number_);
}

void DebugWidget::setPatteNumber(size_t number_)
{
    _ui.PatteNumber->setText(QStringLiteral("Patte #") + QString::number(number_));
}