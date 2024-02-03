#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

/*
 * This example creates a publisher node that publishes a number to a topic.
 */


class NumberPublisher : public rclcpp::Node
{
    public:
        NumberPublisher() : Node("number_publisher")
        {
            // Creates a publisher that published a number to the topic "number"
            // The second argument is the size of the publisher queue
            publisher_ = this->create_publisher<std_msgs::msg::Int64>("number", 10);
            timer_ = this->create_wall_timer(
                std::chrono::seconds(1),
                // Callback
                std::bind(&NumberPublisher::publish_number, this)
            );
            RCLCPP_INFO(
                this->get_logger(),
                "NumberPublisher has been started"
            );
        } 

    private:
        void publish_number()
        {
            // Create a message to send
            auto msg = std_msgs::msg::Int64();
            msg.data = number_;
            publisher_->publish(msg);
        }
        int number_ = 1;
        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    // Create a NumberPublisher object
    auto node = std::make_shared<NumberPublisher>();
    // Spin the node
    rclcpp::spin(node);
    // Shut down ROS2 node
    rclcpp::shutdown();
    return 0;
}
