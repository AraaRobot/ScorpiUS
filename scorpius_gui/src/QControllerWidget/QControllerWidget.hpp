#ifndef QCONTROLLER_WIDGET_HPP
#define QCONTROLLER_WIDGET_HPP

#include <QWidget>
#include <QPainter>

class QControllerWidget : public QWidget
{
    Q_OBJECT

  public:
    QControllerWidget(QWidget* parent_ = nullptr);

  protected:
    void paintEvent(QPaintEvent*) override;

  private:
    QPixmap _baseSVG;
};

#endif  // QCONTROLLER_WIDGET_HPP