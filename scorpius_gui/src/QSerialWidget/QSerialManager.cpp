#include "QSerialManager.hpp"

QSerialManager::QSerialManager(std::shared_ptr<rclcpp::Node> node_, QWidget* parent):
    QWidget(parent),
    _node(node_)
{


}

QSerialManager::~QSerialManager()
{

}