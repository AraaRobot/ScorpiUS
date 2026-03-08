#include "QControllerManager.hpp"

QControllerManager::QControllerManager(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_),
    _node(node_)
{
    _widget = new QControllerWidget(node_, this);
    _client = _node->create_client<scorpius_main::srv::JoyConfig>(SERVICE_NAME);

    this->setupUI();
    this->setupLayout();
    this->setDeadzone();

    connect(_deadzoneBox, &QDoubleSpinBox::valueChanged, this, &QControllerManager::setDeadzone);
}

QControllerManager::~QControllerManager()
{
    _client.reset();
}

void QControllerManager::setupUI()
{
    _deadzoneBox = new QDoubleSpinBox(this);
    _deadzoneBox->setRange(0.0, 1.0);
    _deadzoneBox->setSingleStep(0.01);
    _deadzoneBox->setDecimals(2);
    _deadzoneBox->setValue(JOYSTICK_DEADZONE_DEFAULT);

    _deadzoneLabel = new QLabel(this);
    _deadzoneLabel->setText("Deadzone");

    _controllerSelectBox = new QComboBox(this);
    _controllerSelectBox->addItem("PS4", "PS4");

    _controllerSelectLabel = new QLabel;
    _controllerSelectLabel->setText("Controller");
};

void QControllerManager::setupLayout()
{
    _verticalLayout = new QVBoxLayout;
    _horizontalLayout = new QHBoxLayout;

    _horizontalLayout->addWidget(_deadzoneLabel);
    _horizontalLayout->addWidget(_deadzoneBox);
    _horizontalLayout->addStretch();
    _horizontalLayout->addWidget(_controllerSelectLabel);
    _horizontalLayout->addWidget(_controllerSelectBox);

    _verticalLayout->addWidget(_widget);
    _verticalLayout->addLayout(_horizontalLayout);

    this->setLayout(_verticalLayout);
}

void QControllerManager::setDeadzone()
{
    _widget->setDeadzone(_deadzoneBox->value());
    rclcpp::Client<scorpius_main::srv::JoyConfig>::WeakPtr weakClient = _client;
    float deadzone = _deadzoneBox->value();

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