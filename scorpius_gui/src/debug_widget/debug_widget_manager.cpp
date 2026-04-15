#include "debug_widget_manager.hpp"

DebugWidgetManager::DebugWidgetManager(std::shared_ptr<rclcpp::Node> node_, QWidget* parent):
    QWidget(parent),
    _node(node_)

{
    _grid = new QGridLayout(this);
    _grid->setContentsMargins(0, 0, 0, 0);
    _grid->setSpacing(6);

    for (int r = 0; r < ROWS; ++r)
    {
        for (int c = 0; c < COLS; ++c)
        {
            _debugWidgets[r * COLS + c] = std::make_unique<DebugWidget>(this, aNumberToLetter[r * COLS + c]);
            _grid->addWidget(_debugWidgets[r * COLS + c].get(), r, c);
        }
    }

    _stepLabel = new QLabel(this);
    // _tailLabel = new QLabel(this);

    _stepLabel->setAlignment(Qt::AlignHCenter);
    _stepLabel->setMaximumHeight(LABEL_FONT_SIZE * 2);
    _stepLabel->setText(QStringLiteral("Step:    ?    mm"));

    // _tailLabel->setAlignment(Qt::AlignHCenter);
    // _tailLabel->setMaximumHeight(LABEL_FONT_SIZE * 2);
    // _tailLabel->setText(QStringLiteral("Tail Angle:    ?    deg"));
    
    QFont stepLabelFont = _stepLabel->font();
    stepLabelFont.setPointSize(LABEL_FONT_SIZE);
    _stepLabel->setFont(stepLabelFont);

    // QFont tailLabelFont = _tailLabel->font();
    // tailLabelFont.setPointSize(LABEL_FONT_SIZE);
    // _tailLabel->setFont(tailLabelFont);


    _grid->setRowStretch(4, 8);
    _grid->addWidget(_stepLabel, 4, 0, 1, 2);
    // _grid->addWidget(_tailLabel, 4, 4, 1, 2);

    setLayout(_grid);

    _sub_teleop = _node->create_subscription<scorpius_main::msg::ServoAngles>("/scorpius/teleop",
                                                                              10,
                                                                              [this](const scorpius_main::msg::ServoAngles& msg_)
                                                                              {
                                                                                  this->CB_subTeleop(msg_);
                                                                              });
    _sub_step = _node->create_subscription<scorpius_main::msg::Step>("/scorpius/teleop/step",
                                                                       10,
                                                                       [this](const scorpius_main::msg::Step& msg_)
                                                                       {
                                                                           this->CB_subStep(msg_);
                                                                       });
}

DebugWidgetManager::~DebugWidgetManager()
{
    _sub_teleop.reset();
    _sub_step.reset();
}

void DebugWidgetManager::CB_subTeleop(const scorpius_main::msg::ServoAngles& msg_)
{
    emit _debugWidgets[letterToIndex('A')]->setAngleHorizontalSignal(msg_.horiz_a);
    emit _debugWidgets[letterToIndex('A')]->setAngleVerticalSignal(msg_.vert_a);
    emit _debugWidgets[letterToIndex('B')]->setAngleHorizontalSignal(msg_.horiz_b);
    emit _debugWidgets[letterToIndex('B')]->setAngleVerticalSignal(msg_.vert_b);
    emit _debugWidgets[letterToIndex('C')]->setAngleHorizontalSignal(msg_.horiz_c);
    emit _debugWidgets[letterToIndex('C')]->setAngleVerticalSignal(msg_.vert_c);
    emit _debugWidgets[letterToIndex('D')]->setAngleHorizontalSignal(msg_.horiz_d);
    emit _debugWidgets[letterToIndex('D')]->setAngleVerticalSignal(msg_.vert_d);
    emit _debugWidgets[letterToIndex('E')]->setAngleHorizontalSignal(msg_.horiz_e);
    emit _debugWidgets[letterToIndex('E')]->setAngleVerticalSignal(msg_.vert_e);
    emit _debugWidgets[letterToIndex('F')]->setAngleHorizontalSignal(msg_.horiz_f);
    emit _debugWidgets[letterToIndex('F')]->setAngleVerticalSignal(msg_.vert_f);
   // _tailLabel->setText(QStringLiteral("Tail Angle:    ") + QString::number(msg_.))
}

void DebugWidgetManager::CB_subStep(const scorpius_main::msg::Step& msg_)
{
    _stepLabel->setText(QStringLiteral("Step:    ") + QString::number(msg_.step) + QStringLiteral("    mm"));
}