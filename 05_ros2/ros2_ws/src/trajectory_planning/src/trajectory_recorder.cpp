#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
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

struct ErrorStats
{
    double hip_squared = 0.0;
    double knee_squared = 0.0;
    std::size_t count = 0;
};

void printSummary(const std::string& phase, const ErrorStats& stats)
{
    if (stats.count == 0)
    {
        std::cout << phase << " RMSE: no samples\n";
        return;
    }
    double hip_rmse = std::sqrt(stats.hip_squared / static_cast<double>(stats.count));
    double knee_rmse = std::sqrt(stats.knee_squared / static_cast<double>(stats.count));
    
    std::cout << std::setprecision(10) << phase
              << " RMSE: hip=" << hip_rmse << " knee=" << knee_rmse
              << " n=" << stats.count << '\n';
}
}  // namespace

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("trajectory_recorder");
    std::string algorithm = node->declare_parameter<std::string>("algorithm", "quintic");
    JointTrajectory hip, knee;
    if (!knownAlgorithm(algorithm) ||
        !prepareJoint(node, "hip_", 0.4, algorithm, hip) ||
        !prepareJoint(node, "knee_", -0.3, algorithm, knee))
    {
        std::cerr << "Invalid recorder trajectory parameters.\n";
        rclcpp::shutdown();
        return 1;
    }
    double duration = std::max(hip.parameters.duration, knee.parameters.duration);
    double hold_time = node->declare_parameter<double>("hold_time", 2.0);
    std::string output_file = node->declare_parameter<std::string>(
        "output_file", "/tmp/trajectory_system.csv");
    if (!std::isfinite(hold_time) || hold_time < 0.0 || output_file.empty())
    {
        std::cerr << "Invalid hold_time or output_file.\n";
        rclcpp::shutdown();
        return 1;
    }
    auto start = motionStart(node);
    std::filesystem::path path(output_file);
    std::error_code file_error;
    bool exists = std::filesystem::exists(path, file_error);
    if (file_error || exists)
    {
        std::cerr << "Output path already exists or cannot be checked: " << output_file << '\n';
        rclcpp::shutdown();
        return 1;
    }
    if (path.has_parent_path())
        std::filesystem::create_directories(path.parent_path(), file_error);
    std::ofstream file(output_file);
    if (file_error || !file)
    {
        std::cerr << "Cannot open output file: " << output_file << '\n';
        rclcpp::shutdown();
        return 1;
    }
    file.imbue(std::locale::classic());
    file << std::setprecision(std::numeric_limits<double>::max_digits10);
    file << "time,state_stamp,hip_target,hip_actual,hip_error,hip_torque,"
            "knee_target,knee_actual,knee_error,knee_torque\n";

    double hip_target = 0.0, knee_target = 0.0;
    double hip_torque = 0.0, knee_torque = 0.0;
    bool hip_target_ready = false, knee_target_ready = false;
    bool hip_torque_ready = false, knee_torque_ready = false;
    bool finished = false;
    ErrorStats motion, holding;
    std::size_t rows = 0;

    auto hip_target_sub = node->create_subscription<std_msgs::msg::Float64>(
        "/hip/target_angle", 10, [&](const std_msgs::msg::Float64& message)
        { hip_target = message.data; hip_target_ready = true; });
    auto knee_target_sub = node->create_subscription<std_msgs::msg::Float64>(
        "/knee/target_angle", 10, [&](const std_msgs::msg::Float64& message)
        { knee_target = message.data; knee_target_ready = true; });
    auto hip_torque_sub = node->create_subscription<std_msgs::msg::Float64>(
        "/hip/torque_cmd", 10, [&](const std_msgs::msg::Float64& message)
        { hip_torque = message.data; hip_torque_ready = true; });
    auto knee_torque_sub = node->create_subscription<std_msgs::msg::Float64>(
        "/knee/torque_cmd", 10, [&](const std_msgs::msg::Float64& message)
        { knee_torque = message.data; knee_torque_ready = true; });

    auto state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, [&](const sensor_msgs::msg::JointState& message)
    {
        if (finished || !hip_target_ready || !knee_target_ready ||
            !hip_torque_ready || !knee_torque_ready)
            return;
        if (message.name.size() != 2 || message.position.size() != 2 ||
            message.name[0] != "hip_joint" || message.name[1] != "knee_joint")
            return;
        double time = std::chrono::duration<double>(Clock::now() - start).count();
        if (time > duration + hold_time)
        {
            file.flush();
            printSummary("motion", motion);
            printSummary("hold", holding);
            std::cout << "CSV complete: " << output_file << " rows=" << rows << '\n';
            finished = true;
            return;
        }
        double hip_actual = message.position[0];
        double knee_actual = message.position[1];
        double hip_error = hip_target - hip_actual;
        double knee_error = knee_target - knee_actual;

        if (time >= 0.0)
        {
            ErrorStats& stats = (time <= duration ? motion : holding);

            stats.hip_squared += hip_error * hip_error;
            stats.knee_squared += knee_error * knee_error;
            ++stats.count;
        }

        double state_stamp = static_cast<double>(message.header.stamp.sec)
                           + 1e-9 * message.header.stamp.nanosec;
        file << time << ',' << state_stamp << ','
             << hip_target << ',' << hip_actual << ',' << hip_error << ',' << hip_torque << ','
             << knee_target << ',' << knee_actual << ',' << knee_error << ',' << knee_torque << '\n';
        ++rows;
        if (rows % 100 == 0)
            file.flush();
    });

    std::cout << "Recording latest received values: " << output_file
              << "; motion window=" << duration << " s; hold=" << hold_time << " s\n";
    rclcpp::spin(node);
    file.flush();
    if (!finished)
    {
        printSummary("partial motion", motion);
        printSummary("partial hold", holding);
    }
    rclcpp::shutdown();
    return file ? 0 : 1;
}
