#include "QControllerManager.hpp"

QControllerManager::QControllerManager(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_),
    _node(node_)
{
    setAttribute(Qt::WA_StyledBackground, true);
    setAutoFillBackground(false);

    _widget = new QControllerWidget(node_, ps4Profile(), this);
    _widget->setObjectName("controllerWidget");
    _client = _node->create_client<scorpius_main::srv::JoyConfig>(SERVICE_NAME);

    this->setupUI();
    this->setupLayout();
    this->setStyleSheet("QControllerManager { background: transparent; }"
                        "#deadzoneLabel, #controllerSelectLabel, #deadzoneBox, #controllerSelectBox {"
                        "    background-color: rgba(255, 255, 255, 0.7);"
                        "    color: black;"
                        "}"
                        "#deadzoneBox, #controllerSelectBox {"
                        "    border: 1px solid rgba(0, 0, 0, 0.15);"
                        "}");
    this->setDeadzone();

    connect(_deadzoneBox, &QDoubleSpinBox::valueChanged, this, &QControllerManager::setDeadzone);

    connect(_controllerSelectBox,
            &QComboBox::currentIndexChanged,
            this,
            [this]()
            {
                sControllerProfile p;
                if (_controllerSelectBox->currentData() == "PS4")
                {
                    p = ps4Profile();
                    this->setControllerType("DS5");
                }
                else if (_controllerSelectBox->currentData() == "Xbox")
                {
                    p = xboxProfile();
                    this->setControllerType("Xbox");
                }
                this->_widget->setProfile(p);
            });
}

QControllerManager::~QControllerManager()
{
    _client.reset();
}

void QControllerManager::setupUI()
{
    _deadzoneBox = new QDoubleSpinBox(this);
    _deadzoneBox->setObjectName("deadzoneBox");
    _deadzoneBox->setRange(0.0, 1.0);
    _deadzoneBox->setSingleStep(0.01);
    _deadzoneBox->setDecimals(2);
    _deadzoneBox->setValue(JOYSTICK_DEADZONE_DEFAULT);

    _deadzoneLabel = new QLabel(this);
    _deadzoneLabel->setObjectName("deadzoneLabel");
    _deadzoneLabel->setText("Deadzone");

    _controllerSelectBox = new QComboBox(this);
    _controllerSelectBox->setObjectName("controllerSelectBox");
    _controllerSelectBox->addItem("PS4", "PS4");
    _controllerSelectBox->addItem("Xbox", "Xbox");

    _controllerSelectLabel = new QLabel(this);
    _controllerSelectLabel->setObjectName("controllerSelectLabel");
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

void QControllerManager::setControllerType(const std::string& type_)
{
    const std::string controllerType = type_;

    rclcpp::Client<scorpius_main::srv::JoyConfig>::WeakPtr weakClient = _client;

    _executor.addTask(
        [weakClient, controllerType]()
        {
            rclcpp::Client<scorpius_main::srv::JoyConfig>::SharedPtr client = weakClient.lock();
            if (!client)
            {
                return;
            }

            scorpius_main::srv::JoyConfig::Request::SharedPtr request
                = std::make_shared<scorpius_main::srv::JoyConfig::Request>();
            request->command = scorpius_main::srv::JoyConfig::Request::SET_TYPE;
            request->type = controllerType;
            rclcpp::Client<scorpius_main::srv::JoyConfig>::FutureAndRequestId result = client->async_send_request(request);
        });
}