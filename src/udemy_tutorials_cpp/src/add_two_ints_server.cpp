#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

// This is just to not to have to put the full line when needed. So, that we
// can directly use them (in this case) by doing _1 and _2
using std::placeholders::_1;
using std::placeholders::_2;

class AddTwoIntsServer : public rclcpp::Node
{
    public:
        AddTwoIntsServer() : Node("add_two_ints_server")
        {
            server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
                "add_two_ints",  // Service name
                std::bind(&AddTwoIntsServer::callback_add_two_ints, this, _1, _2));
            RCLCPP_INFO(this->get_logger(), "Service server has been started.");
        }
    private:
        void callback_add_two_ints(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                                   const example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
        {
            response->sum = request->a + request->b;
            RCLCPP_INFO(this->get_logger(), "AddTwoInts: %ld + %ld = %ld", request->a, request->b, response->sum);
        }

        rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
