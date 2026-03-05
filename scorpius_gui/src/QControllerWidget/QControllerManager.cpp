#include "QControllerManager.hpp"

QControllerManager::QControllerManager(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_)
{
    _widget = new QControllerWidget(node_, this);

    this->setupUI();
    this->setupLayout();
}

QControllerManager::~QControllerManager()
{
    // reset ros elements
}

void QControllerManager::setupUI()
{
    _deadzoneLBox = new QDoubleSpinBox(this);
    _deadzoneLBox->setRange(0.0, 1.0);
    _deadzoneLBox->setSingleStep(0.01);
    _deadzoneLBox->setDecimals(2);
    _deadzoneLBox->setValue(JOYSTICK_DEADZONE_MIN);

    _deadzoneRBox = new QDoubleSpinBox(this);
    _deadzoneRBox->setRange(0.0, 1.0);
    _deadzoneRBox->setSingleStep(0.01);
    _deadzoneRBox->setDecimals(2);
    _deadzoneRBox->setValue(JOYSTICK_DEADZONE_MIN);

    _deadzoneLLabel = new QLabel(this);
    _deadzoneLLabel->setText("Left deadzone");

    _deadzoneRLabel = new QLabel(this);
    _deadzoneRLabel->setText("Right deadzone");

    _controllerSelectBox = new QComboBox(this);
    _controllerSelectBox->addItem("PS4", "PS4");

    _controllerSelectLabel = new QLabel;
    _controllerSelectLabel->setText("Controller");
};

void QControllerManager::setupLayout()
{
    _verticalLayout = new QVBoxLayout;
    _horizontalLayout = new QHBoxLayout;

    _horizontalLayout->addWidget(_deadzoneLLabel);
    _horizontalLayout->addWidget(_deadzoneLBox);
    _horizontalLayout->addStretch();
    _horizontalLayout->addWidget(_controllerSelectLabel);
    _horizontalLayout->addWidget(_controllerSelectBox);
    _horizontalLayout->addStretch();
    _horizontalLayout->addWidget(_deadzoneRLabel);
    _horizontalLayout->addWidget(_deadzoneRBox);

    _verticalLayout->addWidget(_widget);
    _verticalLayout->addLayout(_horizontalLayout);

    this->setLayout(_verticalLayout);
}