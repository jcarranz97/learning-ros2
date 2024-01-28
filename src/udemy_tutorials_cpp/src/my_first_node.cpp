#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    // Initialize ROS2 communication
    rclcpp::init(argc, argv);
    // Create node name 'cpp_test'
    auto node = std::make_shared<rclcpp::Node>("cpp_test");

    RCLCPP_INFO(node->get_logger(), "Hello World - CCP Node!!!");
    rclcpp::spin(node);  // This this kind of while true loop
    // Stop/start ROS2 communication
    rclcpp::shutdown();
    return 0;
}
