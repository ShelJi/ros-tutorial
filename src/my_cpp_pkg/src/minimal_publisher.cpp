#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

/*
Publisher example hat periodically sends out a string
*/
class MinimalPublisher : public rclcpp::Node
{
public:
    /*
    Constructor (call Node class constructor with node name)
    */

    MinimalPublisher() : Node("minimal_publisher")
    {
        // create a publisher object
        publisher_ = this->create_publisher<example_interfaces::msg::String>(
            "my_topic",
            10);

        // Periodically call method
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&MinimalPublisher::timer_callback, this));

        // Counter for message send
        counter_ = 0;
    }

private:
    void timer_callback()
    {
        auto msg = example_interfaces::msg::String();
        msg.data = "Hello World " + std::to_string(counter_);

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing: %s", msg.data.c_str());

        counter_++;
    }
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t counter_;
};

int main(int argc, char *argv[])
{
    // Initialize ros2
    rclcpp::init(argc, argv);

    // Initialize and run node
    auto node = std::make_shared<MinimalPublisher>();
    rclcpp::spin(node);

    // cleanup
    rclcpp::shutdown();

    return 0;
}