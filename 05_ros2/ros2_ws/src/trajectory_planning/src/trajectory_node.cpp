#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "trajectory_planning/step_linear.hpp"
#include "trajectory_planning/polynomial.hpp"
#include "trajectory_planning/trapezoidal.hpp"

namespace
{
using Clock = std::chrono::steady_clock;
using namespace trajectory_planning;

struct JointTrajectory
{
    TrajectoryParameters parameters;
    TrapezoidalPlan plan;
};

bool prepareJoint(const std::shared_ptr<rclcpp::Node>& node,
                  const std::string& prefix, double target,
                  const std::string& algorithm, JointTrajectory& joint)
{
    joint.parameters.q0 = node->declare_parameter<double>(prefix + "q0", 0.0);
    joint.parameters.qf = node->declare_parameter<double>(prefix + "qf", target);
    joint.parameters.duration = node->declare_parameter<double>(prefix + "T", 4.0);
    double vmax = node->declare_parameter<double>(prefix + "vmax", 0.5);
    double amax = node->declare_parameter<double>(prefix + "amax", 1.0);
    if (!std::isfinite(joint.parameters.q0) || !std::isfinite(joint.parameters.qf))
        return false;
    if (algorithm == "trapezoidal")
    {
        if (!planTrapezoidal(joint.parameters.q0, joint.parameters.qf, vmax, amax, joint.plan))
            return false;
        joint.parameters.duration = joint.plan.total_time;
        return true;
    }
    return std::isfinite(joint.parameters.duration) && joint.parameters.duration > 0.0;
}

bool knownAlgorithm(const std::string& algorithm)
{
    return algorithm == "step" || algorithm == "linear" || algorithm == "cubic" ||
           algorithm == "quintic" || algorithm == "trapezoidal";
}

// Convert the shared wall-clock start once; elapsed runtime then uses steady_clock.
Clock::time_point motionStart(const std::shared_ptr<rclcpp::Node>& node)
{
    double epoch = node->declare_parameter<double>("start_epoch", 0.0);
    double delay = node->declare_parameter<double>("start_delay", 1.0);
    auto steady_now = Clock::now();
    double wall_now = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    double offset = epoch > 0.0 ? epoch - wall_now : std::max(0.0, delay);
    return steady_now + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(offset));
}

TrajectoryState evaluate(double time, const std::string& algorithm, const JointTrajectory& joint)
{
    if (time < 0.0)
    {
        return {joint.parameters.q0, 0.0, 0.0};
    }

    if (algorithm == "step")
    {
        return stepTrajectory(time, joint.parameters);
    }

    if (algorithm == "linear")
    {
        return linearTrajectory(time, joint.parameters);
    }

    if (algorithm == "cubic")
    {
        return cubicTrajectory(time, joint.parameters);
    }

    if (algorithm == "quintic")
    {
        return quinticTrajectory(time, joint.parameters);
    }

    return sampleTrapezoidal(time, joint.plan);
}

}  // namespace

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("trajectory_node");
    std::string algorithm = node->declare_parameter<std::string>("algorithm", "quintic");
    JointTrajectory hip, knee;
    if (!knownAlgorithm(algorithm) ||
        !prepareJoint(node, "hip_", 0.4, algorithm, hip) ||
        !prepareJoint(node, "knee_", -0.3, algorithm, knee))
    {
        std::cerr << "Invalid trajectory algorithm or parameters.\n";
        rclcpp::shutdown();
        return 1;
    }
    auto start = motionStart(node);
    auto hip_publisher = node->create_publisher<std_msgs::msg::Float64>("/hip/target_angle", 10);
    auto knee_publisher = node->create_publisher<std_msgs::msg::Float64>("/knee/target_angle", 10);

    auto timer = node->create_wall_timer(std::chrono::milliseconds(10), [&]()
    {
        double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
        auto hip_state = evaluate(elapsed, algorithm, hip);
        auto knee_state = evaluate(elapsed, algorithm, knee);
        std_msgs::msg::Float64 hip_message;
        std_msgs::msg::Float64 knee_message;
        hip_message.data = hip_state.position;
        knee_message.data = knee_state.position;
        hip_publisher->publish(hip_message);
        knee_publisher->publish(knee_message);
    });

    std::cout << "Trajectory: " << algorithm
              << "; hip T=" << hip.parameters.duration
              << "; knee T=" << knee.parameters.duration << " s\n";
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
