#include "debug_widget_manager.hpp"

DebugWidgetManager::DebugWidgetManager(QWidget* parent = nullptr):
    QWidget(parent)
{
    _grid = new QGridLayout(this);
    _grid->setContentsMargins(0, 0, 0, 0);
    _grid->setSpacing(6);

    for (int r = 0; r < ROWS; ++r)
    {
        for (int c = 0; c < COLS; ++c)
        {
            auto* widget = new DebugWidget(this);
            _grid->addWidget(widget, r, c);
        }
    }

    setLayout(_grid);
}