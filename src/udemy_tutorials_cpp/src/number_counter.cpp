#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"


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
        } 

    private:
        void number_received(const std_msgs::msg::Int64::SharedPtr msg)
        {
            // Increment the counter
            counter_ += msg->data;
            RCLCPP_INFO(this->get_logger(), "Counter: %d", counter_);
            // Publish the counter value
            auto message = std_msgs::msg::Int64();
            message.data = counter_;
            publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(),
                        "Counter value has been published.");
        }
        int counter_ = 0;
        rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscriber_;
        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
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
