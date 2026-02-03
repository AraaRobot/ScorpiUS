#include <QApplication>
#include <QLabel>
#include <QWidget>

#include <QProcess>
#include "gui_node.hpp"
#include "gui_window.hpp"

int rosThreadFunction(std::shared_ptr<GuiNode> node_);
int guiMain(int argc, char* argv[]);

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<GuiNode> node = std::make_shared<GuiNode>();

    std::jthread rosThread(rosThreadFunction, node);

    int ret = guiMain(argc, argv);
    return ret;
}

int guiMain(int argc, char* argv[])
{
    QApplication app(argc, argv);
    QApplication::setApplicationName("ScorpiUS GUI");

    GuiWindow window;
    window.addTab(new QLabel("Placeholder tab"), "Main");
    window.resize(900, 600);
    window.show();

    int ret = QApplication::exec();
    return ret;
}

int rosThreadFunction(std::shared_ptr<GuiNode> node_)
{
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_);
    executor.spin();

    executor.remove_node(node_);

    return 0;
}