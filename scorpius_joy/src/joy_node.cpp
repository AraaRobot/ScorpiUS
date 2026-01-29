#include "joy_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<JoyNode> node = std::make_shared<JoyNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

JoyNode::JoyNode():
    Node("JoyNode")
{
}