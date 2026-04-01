#include "QSerialManager.hpp"

QSerialManager::QSerialManager(std::shared_ptr<rclcpp::Node> node_, QWidget* parent):
    QWidget(parent),
    _node(node_)
{
    _grid = new QGridLayout(this);
    _button = new QPushButton("Connecter", this);
    _refresh = new QPushButton(this);
    _label = new QLabel(this);
    _box = new QComboBox(this);
    _scroll = new QScrollArea(this);

    _box->addItem("Choose a serial port");

    _box->setItemData(0, Qt::AlignCenter, Qt::TextAlignmentRole);
    _box->setFixedHeight(50);

    _refresh->setText("\u21BB");
    _refresh->setFixedHeight(50);

    _button->setFixedHeight(50);

    _grid->setColumnStretch(0, 0.1);
    _grid->setColumnStretch(1, 4);
    _grid->setColumnStretch(2, 2);
    _grid->setRowStretch(0, 0);
    _grid->setRowStretch(1, 1);

    _grid->addWidget(_refresh, 0, 0);
    _grid->addWidget(_box, 0, 1);
    _grid->addWidget(_button, 0, 2);
    _grid->addWidget(_scroll, 1, 0, 1, 3);

    _label->setText(" ");

    _scroll->setWidget(_label);
    _scroll->setWidgetResizable(true);
    _scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    QFont boxFont = _box->font();
    boxFont.setPointSize(24);
    _box->setFont(boxFont);

    QFont buttonFont = _button->font();
    buttonFont.setPointSize(24);
    _button->setFont(buttonFont);

    QFont refreshFont = _refresh->font();
    refreshFont.setPointSize(24);
    _refresh->setFont(refreshFont);

    QFont labelFont = _label->font();
    labelFont.setPointSize(12);
    _label->setFont(labelFont);

    setLayout(_grid);

    connect(this, &QSerialManager::serialPortsSignal, this, &QSerialManager::serialPortsSlot);
    connect(this, &QSerialManager::serialStatusSignal, this, &QSerialManager::serialStatusSlot);
    connect(_box, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &QSerialManager::comboBoxIndexChanged);
    connect(_button, &QPushButton::clicked, this, &QSerialManager::connectButtonClicked);
    connect(_refresh, &QPushButton::clicked, this, &QSerialManager::refreshButtonClicked);
    connect(this, &QSerialManager::buttonFinishedSignal, this, &QSerialManager::buttonFinishedSlot);

    _srv_ports = _node->create_client<scorpius_main::srv::SerialPorts>("/scorpius/serial/ports");
    _sub_status
        = _node->create_subscription<scorpius_main::msg::SerialStatus>("/scorpius/serial/status",
                                                                       10,
                                                                       [this](const scorpius_main::msg::SerialStatus& msg_)
                                                                       {
                                                                           this->CB_subStatus(msg_);
                                                                       });

    _srv_config = _node->create_client<scorpius_main::srv::SerialConfig>("/scorpius/serial/config");
}

QSerialManager::~QSerialManager() {}

void QSerialManager::CB_subStatus(const scorpius_main::msg::SerialStatus& msg_)
{
    emit this->serialStatusSignal(msg_.message, msg_.ok);
}

void QSerialManager::serialPortsSlot(const std::vector<std::string>& port_name)
{
    _box->clear();

    _box->addItem("Choisir un port série");
    _box->setItemData(0, Qt::AlignCenter, Qt::TextAlignmentRole);

    if (!port_name.empty())
    {
        for (size_t s = 0; s < port_name.size(); s++)
        {
            _box->addItem(QString::fromStdString(port_name.at(s)));
            _box->setItemData(s + 1, Qt::AlignCenter, Qt::TextAlignmentRole);
        }
    }
}

void QSerialManager::serialStatusSlot(const std::string& message, bool is_connected)
{
    const QString port_message = QString::fromStdString(message);

    if (is_connected)
    {
        emit this->writeMessage(port_message);
    }
}

void QSerialManager::writeMessage(const QString& message)
{
    last_message[current_status_index] = message;
    current_status_index = (current_status_index + 1) % last_message.size();

    _label->setText(message + "\n");
    _label->setAlignment(Qt::AlignLeft);
}

void QSerialManager::comboBoxIndexChanged(int index)
{
    if (index == 0)
    {
        _label->setText(" ");
        _label->setAlignment(Qt::AlignCenter);
    }
    else
    {
        _button->show();

        _label->setAlignment(Qt::AlignLeft);
    }
}

void QSerialManager::connectButtonClicked()
{
    if (_box->currentIndex() == 0)
    {
        return;
    }

    rclcpp::Client<scorpius_main::srv::SerialConfig>::WeakPtr weakClient = _srv_config;

    std::weak_ptr<QSerialManager> weakThis = weak_from_this();

    _executor.addTask(
        [weakClient, weakThis]()
        {
            rclcpp::Client<scorpius_main::srv::SerialConfig>::SharedPtr client = weakClient.lock();
            std::shared_ptr widget = weakThis.lock();

            std::shared_ptr<scorpius_main::srv::SerialConfig::Request> request
                = std::make_shared<scorpius_main::srv::SerialConfig::Request>();

            request->port = widget->_box->currentText().toStdString();
            request->timeout = 1;
            request->baud = 115200;

            if (!client || !widget)
            {
                return;
            }

            auto futureAndRequest = client->async_send_request(request);
            std::future<std::shared_ptr<scorpius_main::srv::SerialConfig::Response>> future = std::move(futureAndRequest.future);

            if (future.wait_for(std::chrono::milliseconds(1000)) == std::future_status::ready)
            {
                auto response = future.get();
                if (response->result)
                {
                    emit widget->buttonFinishedSignal("Serial port connected with success\n");
                }
                else
                {
                    emit widget->buttonFinishedSignal("Serial port connection failed\n");
                }
            }
            else
            {
                emit widget->buttonFinishedSignal("Connect button service failed to respond in time\n");
            }
        });
}

void QSerialManager::buttonFinishedSlot(const std::string& message)
{
    const QString port_message = QString::fromStdString(message);
    emit this->writeMessage(port_message);
}

void QSerialManager::refreshButtonClicked()
{
    rclcpp::Client<scorpius_main::srv::SerialPorts>::WeakPtr weakClient = _srv_ports;
    std::weak_ptr<QSerialManager> weakThis = weak_from_this();

    _executor.addTask(
        [weakClient, weakThis]()
        {
            rclcpp::Client<scorpius_main::srv::SerialPorts>::SharedPtr client = weakClient.lock();
            std::shared_ptr widget = weakThis.lock();

            std::shared_ptr<scorpius_main::srv::SerialPorts::Request> request
                = std::make_shared<scorpius_main::srv::SerialPorts::Request>();

            if (!client || !widget)
            {
                return;
            }

            auto futureAndRequest = client->async_send_request(request);

            std::future<std::shared_ptr<scorpius_main::srv::SerialPorts::Response>> future = std::move(futureAndRequest.future);

            if (future.wait_for(std::chrono::milliseconds(1000)) == std::future_status::ready)
            {
                auto response = future.get();

                const std::vector<std::string> ports = response->ports;

                emit widget->serialPortsSignal(ports);
            }
            else
            {
                emit widget->buttonFinishedSignal("Refresh button service failed to respond in time\n");
            }
        });
}