#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("hip_angle_publisher");
    auto publisher = node->create_publisher<std_msgs::msg::Float64>(
        "/hip/target_angle", 10);

    double t = 0.0;
    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(100),
        [&]()
        {
            std_msgs::msg::Float64 message;

            message.data = 0.5 * std::sin(2.0 * M_PI * 0.2 * t);
            publisher->publish(message);

            t += 0.1;
        });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
