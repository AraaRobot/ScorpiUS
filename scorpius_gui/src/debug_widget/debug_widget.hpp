#ifndef DEBUG_WIDGET
#define DEBUG_WIDGET

#include <QWidget>
#include "ui_DebugWidget.h"

class DebugWidget : public QWidget
{
    Q_OBJECT

  public:
    DebugWidget(QWidget* parent_, char id_);
    void setPatteNumber(char id_);
    void setAngleHorizontal(float angle_);
    void setAngleVertical(float angle_);

  private:
    Ui::DebugWidget _ui;
};

#endif  // define DEBUG_WIDGET