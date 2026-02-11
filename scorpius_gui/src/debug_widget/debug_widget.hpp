#ifndef DEBUG_WIDGET
#define DEBUG_WIDGET

#include <QWidget>
#include "ui_DebugWidget.h"

class DebugWidget : public QWidget
{
    Q_OBJECT

  signals:
    void setAngleHorizontalSignal(float angle_);
    void setAngleVerticalSignal(float angle_);

  private slots:
    void setAngleHorizontalSlot(float angle_);
    void setAngleVerticalSlot(float angle_);

  public:
    DebugWidget(QWidget* parent_, char id_);
    void setPatteNumber(char id_);

  private:
    Ui::DebugWidget _ui;
};

#endif  // define DEBUG_WIDGET