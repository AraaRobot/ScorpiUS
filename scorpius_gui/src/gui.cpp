#include "gui_window.hpp"



#include <QApplication>
#include <QLabel>
#include <QProcess>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>

#include <csignal>
#include <thread>

static QApplication* g_app = nullptr;

void handle_sigint(int)
{
    if (g_app)
    {
        g_app->quit();
    }
}

int rosThreadFunction(std::shared_ptr<rclcpp::Node> node_);
int guiMain(int argc, char* argv[], std::shared_ptr<rclcpp::Node> node_);

int main(int argc, char* argv[])
{
    signal(SIGINT, handle_sigint);
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("Scorpius_gui");

    std::jthread rosThread(rosThreadFunction, node);

    int ret = guiMain(argc, argv, node);

    rclcpp::shutdown();

    return ret;
}

int guiMain(int argc, char* argv[], std::shared_ptr<rclcpp::Node> node_)
{
    QApplication app(argc, argv);
    g_app = &app;
    QApplication::setApplicationName("ScorpiUS GUI");

    GuiWindow window(node_);
    window.resize(900, 600);
    window.show();

    int ret = QApplication::exec();
    return ret;
}

int rosThreadFunction(std::shared_ptr<rclcpp::Node> node_)
{
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_);
    executor.spin();

    executor.remove_node(node_);

    return 0;
}