#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"


class SmartphoneNode: public rclcpp::Node
{
    public:
        SmartphoneNode(): Node("smartphone")
        {
            // Create a subscriber on the topic "robot_news"
            subscriber_ = this->create_subscription<example_interfaces::msg::String>(
                "robot_news",
                10,  // Queue size
                std::bind(&SmartphoneNode::callback_robot_news,
                          this,
                          std::placeholders::_1)
            );
            RCLCPP_INFO(this->get_logger(), "Smartophone has been started.");
        }
    private:
        void callback_robot_news(const example_interfaces::msg::String::SharedPtr msg)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Message received: '%s'",
                msg->data.c_str()
            );
        }
        rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_;
};


int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    // Create a SmartphoneNode
    auto node = std::make_shared<SmartphoneNode>();
    // Spin the node
    rclcpp::spin(node);
    // Shut down ROS2
    rclcpp::shutdown();
    return 0;
}
