#include "comm_node.hpp"

// NEED PACKAGE ON UBUNTU 24.04 --> TRANSITION NEEDED BEFORE MAKING THIS NODE

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<CommNode> node = std::make_shared<CommNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

CommNode::CommNode():
    Node("CommNode")
{
}