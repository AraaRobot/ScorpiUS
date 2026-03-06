#include "QControllerManager.hpp"

QControllerManager::QControllerManager(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_)
{
    _widget = new QControllerWidget(node_, this);

    _client = _node->create_client<scorpius_main::srv::JoyConfig>(SERVICE_NAME);

    this->setupUI();
    this->setupLayout();
}

QControllerManager::~QControllerManager()
{
    _client.reset();
}

void QControllerManager::setupUI()
{
    _deadzoneLBox = new QDoubleSpinBox(this);
    _deadzoneLBox->setRange(0.0, 1.0);
    _deadzoneLBox->setSingleStep(0.01);
    _deadzoneLBox->setDecimals(2);
    _deadzoneLBox->setValue(JOYSTICK_DEADZONE_MIN);

    _deadzoneLLabel = new QLabel(this);
    _deadzoneLLabel->setText("Left deadzone");

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

    _verticalLayout->addWidget(_widget);
    _verticalLayout->addLayout(_horizontalLayout);

    this->setLayout(_verticalLayout);
}

void QControllerManager::setDeadzone()
{
    _widget->setDeadzone(_deadzoneLBox->value());
    rclcpp::Client<scorpius_main::srv::JoyConfig>::WeakPtr weakClient = _client;
    float deadzone = _deadzoneLBox->value();

    _executor.addTask(
        [weakClient, deadzone]()
        {
            rclcpp::Client<scorpius_main::srv::JoyConfig>::SharedPtr client = weakClient.lock();
            if (!client)
            {
                return;
            }

            scorpius_main::srv::JoyConfig::Request::SharedPtr request
                = std::make_shared<scorpius_main::srv::JoyConfig::Request>();
            request->command = scorpius_main::srv::JoyConfig::Request::SET_DEADZONE;
            request->deadzone = deadzone;
            rclcpp::Client<scorpius_main::srv::JoyConfig>::FutureAndRequestId result = client->async_send_request(request);
        });
}