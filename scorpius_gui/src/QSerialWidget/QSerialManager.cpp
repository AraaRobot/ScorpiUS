#include "QSerialManager.hpp"

QSerialManager::QSerialManager(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_),
    _node(node_)
{
    _cbPorts = new QComboBox(this);
    _grid = new QGridLayout(this);
    _pbConnect = new QPushButton("Connect", this);
    _pbRefresh = new QPushButton(this);
    _messageDisplay = new QTextEdit(this);

    _cbPorts->addItem("Choose a serial port");

    _cbPorts->setItemData(0, Qt::AlignCenter, Qt::TextAlignmentRole);
    _cbPorts->setFixedHeight(50);

    _pbRefresh->setText("\u21BB");
    _pbRefresh->setFixedHeight(50);

    _pbConnect->setFixedHeight(50);

    _messageDisplay->setReadOnly(true);
    _messageDisplay->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    _messageDisplay->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    _grid->setColumnStretch(0, 0);
    _grid->setColumnStretch(1, 4);
    _grid->setColumnStretch(2, 2);
    _grid->setRowStretch(0, 0);
    _grid->setRowStretch(1, 1);

    _grid->addWidget(_pbRefresh, 0, 0);
    _grid->addWidget(_cbPorts, 0, 1);
    _grid->addWidget(_pbConnect, 0, 2);
    _grid->addWidget(_messageDisplay, 1, 0, 1, 3);

    QFont boxFont = _cbPorts->font();
    boxFont.setPointSize(BOX_FONT_SIZE);
    _cbPorts->setFont(boxFont);

    QFont buttonFont = _pbConnect->font();
    buttonFont.setPointSize(BUTTON_FONT_SIZE);
    _pbConnect->setFont(buttonFont);

    QFont refreshFont = _pbRefresh->font();
    refreshFont.setPointSize(BUTTON_FONT_SIZE);
    _pbRefresh->setFont(refreshFont);

    QFont displayFont = _messageDisplay->font();
    displayFont.setPointSize(DISPLAY_FONT_SIZE);
    _messageDisplay->setFont(displayFont);

    setLayout(_grid);

    qRegisterMetaType<scorpius_main::msg::SerialStatus>("scorpius_main::msg::SerialStatus");

    connect(this, &QSerialManager::serialPortsSignal, this, &QSerialManager::serialPortsSlot, Qt::QueuedConnection);
    connect(this, &QSerialManager::serialStatusSignal, this, &QSerialManager::serialStatusSlot, Qt::QueuedConnection);
    connect(_pbConnect, &QPushButton::clicked, this, &QSerialManager::connectButtonClicked);
    connect(_pbRefresh, &QPushButton::clicked, this, &QSerialManager::refreshButtonClicked);
    connect(this, &QSerialManager::buttonFinishedSignal, this, &QSerialManager::writeMessage, Qt::QueuedConnection);

    _sub_status
        = _node->create_subscription<scorpius_main::msg::SerialStatus>(STATUS_MESSAGE_NAME,
                                                                       10,
                                                                       [this](const scorpius_main::msg::SerialStatus& msg_)
                                                                       {
                                                                           this->CB_subStatus(msg_);
                                                                       });

    _client_ports = _node->create_client<scorpius_main::srv::SerialPorts>(PORTS_SERVICE_NAME);
    _client_config = _node->create_client<scorpius_main::srv::SerialConfig>(CONFIG_SERVICE_NAME);
}

void QSerialManager::CB_subStatus(const scorpius_main::msg::SerialStatus& msg_)
{
    emit this->serialStatusSignal(msg_);
}

void QSerialManager::serialPortsSlot(const QStringList& portName_)
{
    _cbPorts->clear();
    _cbPorts->addItem("Select a port");

    for (const QString& port : portName_)
        _cbPorts->addItem(port);

    _cbPorts->setEditable(true);
    _cbPorts->lineEdit()->setAlignment(Qt::AlignCenter);
    _cbPorts->setEditable(false);
}

void QSerialManager::serialStatusSlot(const scorpius_main::msg::SerialStatus& message_)
{
    writeMessage(QString::fromStdString(message_.message));
}

void QSerialManager::writeMessage(const QString& message_)
{
    QTextCursor cursor = _messageDisplay->textCursor();
    cursor.movePosition(QTextCursor::Start);
    _messageDisplay->setTextCursor(cursor);
    _messageDisplay->insertPlainText(message_ + "\n");

    QTextDocument* doc = _messageDisplay->document();
    while (doc->blockCount() > ARRAY_SIZE)
    {
        cursor.movePosition(QTextCursor::End);
        cursor.select(QTextCursor::BlockUnderCursor);
        cursor.removeSelectedText();
        cursor.deletePreviousChar();
    }

    _messageDisplay->moveCursor(QTextCursor::Start);
}

void QSerialManager::connectButtonClicked()
{
    if (_cbPorts->currentIndex() == 0)
    {
        return;
    }

    std::string selectedPort = _cbPorts->currentText().toStdString();

    rclcpp::Client<scorpius_main::srv::SerialConfig>::WeakPtr weakClient = _client_config;

    QPointer<QSerialManager> thisPtr = this;

    _executor.addTask(
        [weakClient, thisPtr, selectedPort]()
        {
            rclcpp::Client<scorpius_main::srv::SerialConfig>::SharedPtr client = weakClient.lock();

            if (!client || !thisPtr)
            {
                return;
            }

            if (!client->wait_for_service(std::chrono::milliseconds(WAIT_TIME)))
            {
                RCLCPP_ERROR(thisPtr->_node->get_logger(), "SerialConfig's service is not available");
                emit thisPtr->buttonFinishedSignal("Serial config service not available\n");
                return;
            }

            std::shared_ptr<scorpius_main::srv::SerialConfig::Request> request
                = std::make_shared<scorpius_main::srv::SerialConfig::Request>();

            request->port = selectedPort;
            request->timeout = TIMEOUT;
            request->baud = BAUD_RATE;

            auto futureAndRequest = client->async_send_request(request);
            std::future<std::shared_ptr<scorpius_main::srv::SerialConfig::Response>> future = std::move(futureAndRequest.future);

            if (future.wait_for(std::chrono::milliseconds(WAIT_TIME)) == std::future_status::ready)
            {
                auto response = future.get();
                emit thisPtr->buttonFinishedSignal(QString::fromStdString(response->response));
            }
            else
            {
                emit thisPtr->buttonFinishedSignal("Connect button service failed to respond in time\n");
            }
        });
}

void QSerialManager::refreshButtonClicked()
{
    rclcpp::Client<scorpius_main::srv::SerialPorts>::WeakPtr weakClient = _client_ports;
    QPointer<QSerialManager> thisPtr = this;

    _executor.addTask(
        [weakClient, thisPtr]()
        {
            rclcpp::Client<scorpius_main::srv::SerialPorts>::SharedPtr client = weakClient.lock();

            if (!client || !thisPtr)
            {
                return;
            }

            if (!client->wait_for_service(std::chrono::milliseconds(WAIT_TIME)))
            {
                RCLCPP_ERROR(thisPtr->_node->get_logger(), "SerialPort's service is not available");
                emit thisPtr->buttonFinishedSignal("Serial ports service not available\n");
                return;
            }

            std::shared_ptr<scorpius_main::srv::SerialPorts::Request> request
                = std::make_shared<scorpius_main::srv::SerialPorts::Request>();

            auto futureAndRequest = client->async_send_request(request);

            std::future<std::shared_ptr<scorpius_main::srv::SerialPorts::Response>> future = std::move(futureAndRequest.future);

            if (future.wait_for(std::chrono::milliseconds(WAIT_TIME)) == std::future_status::ready)
            {
                auto response = future.get();

                QStringList portList;

                for (const std::string& p : response->ports)
                    portList.append(QString::fromStdString(p));

                emit thisPtr->serialPortsSignal(portList);

                RCLCPP_INFO(thisPtr->_node->get_logger(), "Ports refreshed successfully");
            }
            else
            {
                emit thisPtr->buttonFinishedSignal("Refresh button service failed to respond in time\n");
                RCLCPP_ERROR(thisPtr->_node->get_logger(), "The server did not respond in time");
            }
        });
}