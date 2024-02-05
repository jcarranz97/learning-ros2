#include "rclcpp/rclcpp.hpp"
#include "udemy_tutorials_interfaces/msg/hardware_status.hpp"


class HardwareStatusPublisher : public rclcpp::Node
{
    public:
        HardwareStatusPublisher() : Node("hw_status_publisher")
        {
            publisher_ = this->create_publisher<udemy_tutorials_interfaces::msg::HardwareStatus>(
                "hw_status", 10);
            //This is the constructor of my node
            timer_ = this->create_wall_timer(
                std::chrono::seconds(1),
                //Callback for this timer
                std::bind(&HardwareStatusPublisher::send_hardware_status, this)
            );
            RCLCPP_INFO(this->get_logger(),
                        "HardwareStatusPublisher is ready!");
        } 
    private:
        void send_hardware_status()
        {
            // Create HardwareStatus message to send it
            auto msg = udemy_tutorials_interfaces::msg::HardwareStatus();
            msg.temperature = 35;
            msg.are_motors_ready = false;
            msg.debug_message = "Hello!!!";
            publisher_->publish(msg);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<udemy_tutorials_interfaces::msg::HardwareStatus>::SharedPtr publisher_;
};


int main(int argc, char **argv)
{
    //Initialize ROS2 node
    rclcpp::init(argc, argv);
    // Create a HardwareStatusPublisher
    auto node = std::make_shared<HardwareStatusPublisher>();
    // Sping the node
    rclcpp::spin(node);
    // Shut down ROS2 node
    rclcpp::shutdown();
    return 0;
}
