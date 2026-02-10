#ifndef DEBUG_WIDGET_MANAGER
#define DEBUG_WIDGET_MANAGER

#include <QWidget>
#include <QGridLayout>

#include "debug_widget.hpp"

class DebugWidgetManager : public QWidget
{
    Q_OBJECT

    static constexpr int ROWS = 3;
    static constexpr int COLS = 2;

  public:
    DebugWidgetManager(QWidget* parent);

  private:
    QGridLayout* _grid;
};

#endif  // DEBUG_WIDGET_MANAGER