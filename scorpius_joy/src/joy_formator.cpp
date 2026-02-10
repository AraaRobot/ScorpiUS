#include "joy_formator.hpp"

static constexpr int64_t PUB_FREQ = 10;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<JoyFormator> node = std::make_shared<JoyFormator>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

JoyFormator::JoyFormator():
    Node("JoyFormator")
{
    _sub_joy = this->create_subscription<sensor_msgs::msg::Joy>("raw/joy",
                                                                10,
                                                                [this](const sensor_msgs::msg::Joy msg_)
                                                                {
                                                                    this->joySubscriber_CB(msg_);
                                                                });

    _pub_joyFormat = this->create_publisher<scorpius_main::msg::JoyFormat>("/scorpius/joy", 10);

    _timer_pub = this->create_wall_timer(std::chrono::milliseconds(static_cast<size_t>(1000 / PUB_FREQ)),
                                         [this](void)
                                         {
                                             this->joyPublisher_CB();
                                         });
}

void JoyFormator::joySubscriber_CB(const sensor_msgs::msg::Joy& msg_)
{
    this->_lastMsg = msg_;
}

void JoyFormator::joyPublisher_CB(void)
{
    scorpius_main::msg::JoyFormat msg;
    msg.a = 1;
    msg.b = 1;
    msg.x = 1;
    msg.y = 1;
    _pub_joyFormat->publish(msg);
}