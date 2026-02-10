#ifndef DEBUG_WIDGET
#define DEBUG_WIDGET

#include <QWidget>
#include "ui_DebugWidget.h"


class DebugWidget : public QWidget
{
    Q_OBJECT

  public:
    DebugWidget(QWidget* parent_);

  private:
  Ui::DebugWidget _ui;
};

#endif  // define DEBUG_WIDGET