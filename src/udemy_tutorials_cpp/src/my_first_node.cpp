#include "rclcpp/rclcpp.hpp"

class MyNode: public rclcpp::Node
{
    public:
        MyNode(): Node("cpp_test")  // This is the constructor of rclcpp::Node
        {
            // This is the contructor of MyNode
            RCLCPP_INFO(this->get_logger(), "[MyNode] Hello World CPP Node!");
            timer_ = this->create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&MyNode::timerCallback, this));
        }
    private:
        void timerCallback()
        {
            counter_++;
            RCLCPP_INFO(
                this->get_logger(), "Timer callback (%d)!!!", counter_);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        int counter_ = 0; // Create a counter variable and initialize it to 0
};


int main(int argc, char **argv)
{
    // Initialize ROS2 communication
    rclcpp::init(argc, argv);
    // Create an instance of MyNode
    auto node = std::make_shared<MyNode>();

    rclcpp::spin(node);  // This this kind of while true loop
    // Stop/start ROS2 communication
    rclcpp::shutdown();
    return 0;
}
