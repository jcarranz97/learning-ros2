#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"


/*
 * This example demostrates how to create a simple client node for the 
 * add_two_ints service using the OOP style.
 */

class AddTwoIntsClient : public rclcpp::Node
{
    public:
        AddTwoIntsClient() : Node("add_two_ints_client")
        {
            // Create a client for the add_two_ints service
            //
            // If we do the call directly, it will block the node and we will
            // not be able to do anything else. To avoid this, we can use a
            // thread to call the service.
            // call_add_two_ints_service(1, 2);
            //
            // Create a thread to call the service
            thread1_ = std::thread(std::bind(
                &AddTwoIntsClient::call_add_two_ints_service, this, 3, 4));
            thread2_ = std::thread(std::bind(
                &AddTwoIntsClient::call_add_two_ints_service, this, 5, 6));
        }
        void call_add_two_ints_service(int a, int b)
        {
            auto client = create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
            while(!client->wait_for_service(std::chrono::seconds(1))){
                RCLCPP_INFO(get_logger(), "Waiting for the server to be up...");
            }
            RCLCPP_INFO(get_logger(), "Server is up!!.");
            // Create a request message and set the values entered by the user
            auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
            request->a = a;
            request->b = b;
            // Call the service
            RCLCPP_INFO(get_logger(), "Sending service request:");
            RCLCPP_INFO(get_logger(), "  a: %ld", request->a);
            RCLCPP_INFO(get_logger(), "  b: %ld", request->b);
            auto future = client->async_send_request(request);
            // Wait for the future inside this future even if the node is spinning
            try {
                auto response = future.get();
                RCLCPP_INFO(get_logger(), "  Result = %ld", response->sum);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(get_logger(), "Service call failed");
            }
        }
    private:
        std::thread thread1_;
        std::thread thread2_;
};

int main(int argc, char **argv)
{
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);
    // Create a node
    auto node = std::make_shared<AddTwoIntsClient>();
    // Spin the node and shutdown the ROS 2 system
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
