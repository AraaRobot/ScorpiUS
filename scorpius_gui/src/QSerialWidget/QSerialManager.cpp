#include "QSerialManager.hpp"

QSerialManager::QSerialManager(std::shared_ptr<rclcpp::Node> node_, QWidget* parent):
    QWidget(parent),
    _node(node_)
{
    _grid = new QGridLayout(this);
    _button = new QPushButton("Connecter", this);
    _label = new QLabel(this);
    _box = new QComboBox(this);
    _scroll = new QScrollArea(this);

    _box->addItem("Choisir un port série");

    _box->setItemData(0, Qt::AlignCenter, Qt::TextAlignmentRole);
    _box->setMinimumHeight(50);

    _grid->setColumnStretch(0, 3);
    _grid->setColumnStretch(1, 1);
    _grid->setRowStretch(0, 0);
    _grid->setRowStretch(1, 1);

    _grid->addWidget(_box, 0, 0);
    _grid->addWidget(_button, 0, 1);
    _grid->addWidget(_scroll, 1, 0, 1, 2);

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

    QFont labelFont = _label->font();
    labelFont.setPointSize(12);
    _label->setFont(labelFont);

    setLayout(_grid);

    connect(this, &QSerialManager::serialPortsSignal, this, &QSerialManager::serialPortsSlot);
    connect(this, &QSerialManager::serialStatusSignal, this, &QSerialManager::serialStatusSlot);
    connect(_box, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &QSerialManager::comboBoxIndexChanged);
    connect(_button, &QPushButton::clicked, this, &QSerialManager::pushButtonClicked);

    _sub_ports = _node->create_subscription<scorpius_main::msg::SerialPorts>("/scorpius/serial/ports",
                                                                             10,
                                                                             [this](const scorpius_main::msg::SerialPorts& msg_)
                                                                             {
                                                                                 this->CB_subPorts(msg_);
                                                                             });
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

void QSerialManager::CB_subPorts(const scorpius_main::msg::SerialPorts& msg_)
{
    emit this->serialPortsSignal(msg_.ports);
}

void QSerialManager::CB_subStatus(const scorpius_main::msg::SerialStatus& msg_)
{
    emit this->serialStatusSignal(msg_.message, msg_.ok);
}

void QSerialManager::serialPortsSlot(std::vector<std::string> port_name)
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

void QSerialManager::serialStatusSlot(std::string message, bool is_connected)
{
    _label->clear();
    _label->setAlignment(Qt::AlignLeft);

    const QString port_message = QString::fromStdString(message);

    if (change_connection != is_connected)
    {
        if (is_connected && _box->currentIndex() != 0)
        {
            last_message += "Le port série est connecté\n";
        }
        else if (!is_connected && _box->currentIndex() != 0)
        {
            last_message += "Le port série est déconnecté\n";
        }
        else if (_box->currentIndex() == 0)
        {
            last_message = "";
        }
        change_connection = is_connected;
    }

    if (_box->currentIndex() != 0)
    {
        last_message += port_message + "\n";
        _label->setText(last_message);
    }

    QFont labelFont = _label->font();
    labelFont.setPointSize(24);
    _label->setFont(labelFont);
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

void QSerialManager::pushButtonClicked()
{
    if (_box->currentIndex() == 0)
    {
        return;
    }
}