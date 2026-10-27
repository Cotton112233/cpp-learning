#include <algorithm>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "pd_controller.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("joint_pd_controller");

    node->declare_parameter<double>("hip_kp", 50.0);
    node->declare_parameter<double>("hip_kd", 5.0);
    node->declare_parameter<double>("knee_kp", 50.0);
    node->declare_parameter<double>("knee_kd", 5.0);

    double hip_kp = node->get_parameter("hip_kp").as_double();
    double hip_kd = node->get_parameter("hip_kd").as_double();
    double knee_kp = node->get_parameter("knee_kp").as_double();
    double knee_kd = node->get_parameter("knee_kd").as_double();
    PDController hip_controller(hip_kp, hip_kd);
    PDController knee_controller(knee_kp, knee_kd);

    double hip_target_angle = 0.0;
    double knee_target_angle = 0.0;

    auto hip_torque_publisher = node->create_publisher<std_msgs::msg::Float64>(
        "/hip/torque_cmd", 1);
    auto knee_torque_publisher = node->create_publisher<std_msgs::msg::Float64>(
        "/knee/torque_cmd", 1);

    auto hip_target_subscription = node->create_subscription<std_msgs::msg::Float64>(
        "/hip/target_angle", 10,
        [&](const std_msgs::msg::Float64& message)
        {
            hip_target_angle = message.data;
        });
    auto knee_target_subscription = node->create_subscription<std_msgs::msg::Float64>(
        "/knee/target_angle", 10,
        [&](const std_msgs::msg::Float64& message)
        {
            knee_target_angle = message.data;
        });

    auto joint_state_subscription = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 1,
        [&](const sensor_msgs::msg::JointState& message)
        {

            if (message.name.size() != 2 || message.position.size() != 2 ||
                message.velocity.size() != 2 ||
                message.name[0] != "hip_joint" || message.name[1] != "knee_joint")
            {
                return;
            }

            double hip_angle = message.position[0];
            double hip_velocity = message.velocity[0];
            double knee_angle = message.position[1];
            double knee_velocity = message.velocity[1];

            double hip_torque = hip_controller.calculate(
                hip_target_angle, hip_angle, hip_velocity, 0.0);
            double knee_torque = knee_controller.calculate(
                knee_target_angle, knee_angle, knee_velocity, 0.0);

            hip_torque = std::clamp(hip_torque, -30.0, 30.0);
            knee_torque = std::clamp(knee_torque, -30.0, 30.0);

            std_msgs::msg::Float64 hip_message;
            hip_message.data = hip_torque;
            hip_torque_publisher->publish(hip_message);

            std_msgs::msg::Float64 knee_message;
            knee_message.data = knee_torque;
            knee_torque_publisher->publish(knee_message);
        });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
