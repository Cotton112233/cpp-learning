#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("hip_angle_subscriber");
    auto subscription = node->create_subscription<std_msgs::msg::Float64>(
        "/hip/target_angle", 10,
        [](const std_msgs::msg::Float64& message)
        {
            std::cout << message.data << std::endl;
        });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
