#include "QSerialManager.hpp"

QSerialManager::QSerialManager(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_),
    _node(node_)
{
    _grid = new QGridLayout(this);
    _pb_connect = new QPushButton("Connect", this);
    _pb_refresh = new QPushButton(this);
    _message_display = new QTextEdit(this);
    _combo_box = new QComboBox(this);

    _combo_box->addItem("Choose a serial port");

    _combo_box->setItemData(0, Qt::AlignCenter, Qt::TextAlignmentRole);
    _combo_box->setFixedHeight(50);

    _pb_refresh->setText("\u21BB");
    _pb_refresh->setFixedHeight(50);

    _pb_connect->setFixedHeight(50);

    _message_display->setReadOnly(true);
    _message_display->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    _message_display->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    _grid->setColumnStretch(0, 0);
    _grid->setColumnStretch(1, 4);
    _grid->setColumnStretch(2, 2);
    _grid->setRowStretch(0, 0);
    _grid->setRowStretch(1, 1);

    _grid->addWidget(_pb_refresh, 0, 0);
    _grid->addWidget(_combo_box, 0, 1);
    _grid->addWidget(_pb_connect, 0, 2);
    _grid->addWidget(_message_display, 1, 0, 1, 3);

    QFont boxFont = _combo_box->font();
    boxFont.setPointSize(BOX_FONT_SIZE);
    _combo_box->setFont(boxFont);

    QFont buttonFont = _pb_connect->font();
    buttonFont.setPointSize(BUTTON_FONT_SIZE);
    _pb_connect->setFont(buttonFont);

    QFont refreshFont = _pb_refresh->font();
    refreshFont.setPointSize(BUTTON_FONT_SIZE);
    _pb_refresh->setFont(refreshFont);

    QFont displayFont = _message_display->font();
    displayFont.setPointSize(DISPLAY_FONT_SIZE);
    _message_display->setFont(displayFont);

    setLayout(_grid);

    qRegisterMetaType<scorpius_main::msg::SerialStatus>("scorpius_main::msg::SerialStatus");

    connect(this, &QSerialManager::serialPortsSignal, this, &QSerialManager::serialPortsSlot, Qt::QueuedConnection);
    connect(this, &QSerialManager::serialStatusSignal, this, &QSerialManager::serialStatusSlot, Qt::QueuedConnection);
    connect(_pb_connect, &QPushButton::clicked, this, &QSerialManager::connectButtonClicked);
    connect(_pb_refresh, &QPushButton::clicked, this, &QSerialManager::refreshButtonClicked);
    connect(this, &QSerialManager::buttonFinishedSignal, this, &QSerialManager::buttonFinishedSlot, Qt::QueuedConnection);

    _sub_status
        = _node->create_subscription<scorpius_main::msg::SerialStatus>(STATUS_MESSAGE_NAME,
                                                                       10,
                                                                       [this](const scorpius_main::msg::SerialStatus& msg_)
                                                                       {
                                                                           this->CB_subStatus(msg_);
                                                                       });

    _srv_ports = _node->create_client<scorpius_main::srv::SerialPorts>(PORTS_SERVICE_NAME);
    _srv_config = _node->create_client<scorpius_main::srv::SerialConfig>(CONFIG_SERVICE_NAME);
}

void QSerialManager::CB_subStatus(const scorpius_main::msg::SerialStatus& msg_)
{
    emit this->serialStatusSignal(msg_);
}

void QSerialManager::serialPortsSlot(const QStringList& portName_)
{
    _combo_box->clear();
    _combo_box->addItem("Select a port");

    for (const QString& port : portName_)
        _combo_box->addItem(port);

    _combo_box->setEditable(true);
    _combo_box->lineEdit()->setAlignment(Qt::AlignCenter);
    _combo_box->setEditable(false);
}

void QSerialManager::serialStatusSlot(const scorpius_main::msg::SerialStatus& message_)
{
    const QString portMessage = QString::fromStdString(message_.message);

    emit this->writeMessage(portMessage);
}

void QSerialManager::writeMessage(const QString& message_)
{
    QTextCursor cursor = _message_display->textCursor();
    cursor.movePosition(QTextCursor::Start);
    _message_display->setTextCursor(cursor);
    _message_display->insertPlainText(message_ + "\n");

    QTextDocument* doc = _message_display->document();
    while (doc->blockCount() > ARRAY_SIZE)
    {
        cursor.movePosition(QTextCursor::End);
        cursor.select(QTextCursor::BlockUnderCursor);
        cursor.removeSelectedText();
        cursor.deletePreviousChar();
    }

    _message_display->moveCursor(QTextCursor::Start);
}

void QSerialManager::connectButtonClicked()
{
    if (_combo_box->currentIndex() == 0)
    {
        return;
    }

    std::string selectedPort = _combo_box->currentText().toStdString();

    rclcpp::Client<scorpius_main::srv::SerialConfig>::WeakPtr weakClient = _srv_config;

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
            request->timeout = 1;
            request->baud = 115200;

            auto futureAndRequest = client->async_send_request(request);
            std::future<std::shared_ptr<scorpius_main::srv::SerialConfig::Response>> future = std::move(futureAndRequest.future);

            if (future.wait_for(std::chrono::milliseconds(WAIT_TIME)) == std::future_status::ready)
            {
                auto response = future.get();
                if (response->result)
                {
                    emit thisPtr->buttonFinishedSignal("Serial port connected with success\n");
                }
                else
                {
                    emit thisPtr->buttonFinishedSignal("Serial port connection failed\n");
                }
            }
            else
            {
                emit thisPtr->buttonFinishedSignal("Connect button service failed to respond in time\n");
            }
        });
}

void QSerialManager::buttonFinishedSlot(const QString& message_)
{
    const QString portMessage = message_;
    emit this->writeMessage(portMessage);
}

void QSerialManager::refreshButtonClicked()
{
    rclcpp::Client<scorpius_main::srv::SerialPorts>::WeakPtr weakClient = _srv_ports;
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

            if (future.wait_for(std::chrono::milliseconds(2 * WAIT_TIME)) == std::future_status::ready)
            {
                auto response = future.get();
                
                QStringList portList;

                for (const std::string& p : response->ports)
                    portList.append(QString::fromStdString(p));

                emit thisPtr->serialPortsSignal(portList);
                emit thisPtr->serialPortsSignal(std::move(portList));

                RCLCPP_INFO(thisPtr->_node->get_logger(), "Ports refreshed successfully");
            }
            else
            {
                emit thisPtr->buttonFinishedSignal("Refresh button service failed to respond in time\n");
                RCLCPP_ERROR(thisPtr->_node->get_logger(), "The server did not respond in time");
            }
        });
}