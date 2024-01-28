#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"


class RobotNewsStationNode: public rclcpp::Node
{
    public:
        RobotNewsStationNode(): Node("robot_news_station"), robot_name_("R2D2")
        {
            // Create a publisher for the robot_news topic
            publisher_ = this->create_publisher<example_interfaces::msg::String>("robot_news", 10);
            timer_ = this->create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&RobotNewsStationNode::publish_news, this)
            );
            RCLCPP_INFO(
                this->get_logger(), 
                "Robot News Station has been started"
            );
        }

    private:
        void publish_news()
        {
            // Create a message
            auto msg = example_interfaces::msg::String();
            msg.data = 
                std::string("Hi, This is ") + 
                robot_name_ + 
                std::string(" from the Robot News Station");
            publisher_->publish(msg);
        }
        std::string robot_name_;
        rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    // Create a RobotNewsStationNode object
    auto node = std::make_shared<RobotNewsStationNode>();
    // Spin the node
    rclcpp::spin(node);  // This is like a while true loop
    // Shut down ROS2 node
    rclcpp::shutdown();
    return 0;
}
