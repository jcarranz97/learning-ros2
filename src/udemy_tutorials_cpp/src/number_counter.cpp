#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class NumberCounter : public rclcpp::Node
{
    public:
        NumberCounter(): Node("number_counter")
        {
            // Create subscriber to listen to the topic "number"
            // The callback function is called when a message is received
            // The message is of type std_msgs::msg::Int64
            subscriber_ = this->create_subscription<std_msgs::msg::Int64>(
                "number",
                10,  // Queue size
                std::bind(&NumberCounter::number_received,
                          this,
                          std::placeholders::_1)
            );
            RCLCPP_INFO(this->get_logger(), "Number counter has been started.");
            publisher_ = this->create_publisher<std_msgs::msg::Int64>(
                "number_count",
                10
            );
            service_ = this->create_service<example_interfaces::srv::SetBool>(
                "reset_counter",
                std::bind(&NumberCounter::callback_reset_counter, this, _1, _2)
            );
        } 

    private:
        void number_received(const std_msgs::msg::Int64::SharedPtr msg)
        {
            // Increment the counter
            counter_ += msg->data;
            RCLCPP_INFO(this->get_logger(), "Counter: %d", counter_);
            this->publish_counter();
        }

        void publish_counter()
        {
            // Publish the counter value
            auto message = std_msgs::msg::Int64();
            message.data = counter_;
            publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(),
                        "Counter value has been published (from function).");
        }

        void callback_reset_counter(const example_interfaces::srv::SetBool::Request::SharedPtr request,
                                    const example_interfaces::srv::SetBool::Response::SharedPtr response)
        {
            if (request->data)
            {
                counter_ = 0;
                RCLCPP_INFO(this->get_logger(), "Counter has been reset.");
                response->success = true;
                response->message = "Counter has been reset.";
            }
            else
            {
                response->success = false;
                response->message = "Counter has not been reset.";
            }
        }
        int counter_ = 0;
        rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscriber_;
        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
        rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr service_;
};


int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    // Create a NumberCounter node
    auto node = std::make_shared<NumberCounter>();
    // Spin the node
    rclcpp::spin(node);
    // Shut down ROS2
    rclcpp::shutdown();
    return 0;
}
