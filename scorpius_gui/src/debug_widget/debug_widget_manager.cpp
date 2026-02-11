#include "debug_widget_manager.hpp"

DebugWidgetManager::DebugWidgetManager(std::shared_ptr<rclcpp::Node> node_, QWidget* parent = nullptr): QWidget(parent),
    _node(node_)
    
{
    _grid = new QGridLayout(this);
    _grid->setContentsMargins(0, 0, 0, 0);
    _grid->setSpacing(6);

    for (int r = 0; r < ROWS; ++r)
    {
        for (int c = 0; c < COLS; ++c)
        {
            _debugWidgets[r * COLS + c] = std::make_unique<DebugWidget>(this, r * COLS + c + 1);
            _grid->addWidget(_debugWidgets[r * COLS + c].get(), r, c);
        }
    }

    setLayout(_grid);
}