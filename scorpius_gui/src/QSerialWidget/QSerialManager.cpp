#include "QSerialManager.hpp"

QSerialManager::QSerialManager(std::shared_ptr<rclcpp::Node> node_, QWidget* parent):
    QWidget(parent),
    _node(node_)
{
    _grid = new QGridLayout(this);

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

    switch (_box->currentIndex())
    {
        case 0:
            _label->setText("Choisir un port série");
            _label->setAlignment(Qt::AlignCenter);
            _grid->addWidget(_label, 1, 0);
            break;
        case 1:
            _button = new QRadioButton("Serial Port 2", this);
            break;
        default:
            _button = new QRadioButton("No Serial Port Selected", this);
            break;
    }

    setLayout(_grid);
}

QSerialManager::~QSerialManager() {}