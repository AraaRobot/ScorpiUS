#include "gui_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<GuiNode> node = std::make_shared<GuiNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

GuiNode::GuiNode():
    Node("GuiNode")
{
}