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

    node->declare_parameter<double>("amplitude", 0.5);
    node->declare_parameter<double>("frequency", 0.2);//外部启动参数入口
    double amplitude = node->get_parameter("amplitude").as_double();
    double frequency = node->get_parameter("frequency").as_double();
    
    double t = 0.0;
    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(100),
        [&]()
        {
            std_msgs::msg::Float64 message;

            message.data = amplitude * std::sin(2.0 * M_PI * frequency * t);
            publisher->publish(message);

            t += 0.1;
        });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
