#include "QControllerWidget.hpp"

QControllerWidget::QControllerWidget(QWidget* parent_) : QWidget(parent_)
{
    _baseSVG.load(":/images/Dualshock_4_Layout.svg");
    this->update();
}

void QControllerWidget::paintEvent(QPaintEvent*)
    {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing);
        p.setRenderHint(QPainter::SmoothPixmapTransform);

        QPixmap scaled = _baseSVG.scaled(size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);

        QPoint topLeft((width() - scaled.width()) / 2, (height() - scaled.height()) / 2);

        QRect imageRect(topLeft, scaled.size());

        p.drawPixmap(imageRect, scaled);
    }