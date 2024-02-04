#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("add_two_ints_client_no_oop");

    // Create client without OOP
    auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    RCLCPP_INFO(node->get_logger(), "AddTwoInts client (no opp) initialized");
    
    // Wait for server to be up
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(node->get_logger(), "Service not available, waiting again...");
    }
    RCLCPP_INFO(node->get_logger(), "Service is available");
    // Create request to be send with node
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();

    // Set request values
    request->a = 3;
    request->b = 8;

    // Send request to server
    auto future = client->async_send_request(request);
    RCLCPP_INFO(node->get_logger(), "Request sent");
    RCLCPP_INFO(node->get_logger(),
                "  Request: %ld + %ld", request->a, request->b);
    // Wait for response using future
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "  Sum: %ld", future.get()->sum);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Service call failed");
    } 
    // Shutdown the node
    rclcpp::shutdown();
    return 0;
}
