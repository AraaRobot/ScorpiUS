#include "QSerialManager.hpp"

QSerialManager::QSerialManager(std::shared_ptr<rclcpp::Node> node_, QWidget* parent):
    QWidget(parent),
    _node(node_)
{
    _grid = new QGridLayout(this);

    _button = new QPushButton("Connecter", this);

    _box = new QComboBox(this);
    _box->addItem("N/A");

    _box->setItemData(0, Qt::AlignCenter, Qt::TextAlignmentRole);

    QFont boxFont = _box->font();
    boxFont.setPointSize(24);
    _box->setFont(boxFont);

    _grid->addWidget(_box, 0, 0);

    _label = new QLabel(this);

    QFont labelFont = _label->font();
    labelFont.setPointSize(28);
    _label->setFont(labelFont);

    _label->setText("Choisir un port série");
    _label->setAlignment(Qt::AlignCenter);
    _grid->addWidget(_label, 1, 0);

    setLayout(_grid);

    connect(this, &QSerialManager::serialPortsSignal, this, &QSerialManager::serialPortsSlot);
    connect(_box, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &QSerialManager::comboBoxIndexChanged);

    _sub_ports = _node->create_subscription<scorpius_main::msg::SerialPorts>("/scorpius/serial/ports",
                                                                             10,
                                                                             [this](const scorpius_main::msg::SerialPorts& msg_)
                                                                             {
                                                                                 this->CB_subPorts(msg_);
                                                                             });
}

QSerialManager::~QSerialManager() {}

void QSerialManager::CB_subPorts(const scorpius_main::msg::SerialPorts& msg_)
{
    emit this->serialPortsSignal(msg_.ports);
}

void QSerialManager::serialPortsSlot(std::vector<std::string> port_name)
{
    _box->clear();

    _box->addItem("N/A");
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

void QSerialManager::comboBoxIndexChanged(int index)
{
    if (index == 0)
    {
        _button->hide();

        QFont labelFont = _label->font();
        labelFont.setPointSize(28);
        _label->setFont(labelFont);

        _label->setText("Choisir un port série");
        _label->setAlignment(Qt::AlignCenter);
        _grid->addWidget(_label, 1, 0);
    }
    else
    {
        _label->clear();

        _button->show();

        _grid->addWidget(_button, 1, 0);

        QFont buttonFont = _button->font();
        buttonFont.setPointSize(40);
        _button->setFont(buttonFont);
        
    }
}