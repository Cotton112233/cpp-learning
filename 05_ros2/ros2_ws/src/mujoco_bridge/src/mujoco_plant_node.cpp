#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include <mujoco/mujoco.h>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mujoco_plant_node");

    std::string model_path =
        ament_index_cpp::get_package_share_directory("mujoco_bridge") +
        "/models/two_dof.xml";

    char error[1024] = {};
    mjModel* model = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    if (model == nullptr)
    {
        std::cout << "Failed to load model: " << error << '\n';
        rclcpp::shutdown();
        return 1;
    }

    mjData* data = mj_makeData(model);

    int hip_joint_id = mj_name2id(model, mjOBJ_JOINT, "hip_joint");
    int knee_joint_id = mj_name2id(model, mjOBJ_JOINT, "knee_joint");
    int hip_motor_id = mj_name2id(model, mjOBJ_ACTUATOR, "hip_motor");
    int knee_motor_id = mj_name2id(model, mjOBJ_ACTUATOR, "knee_motor");
    if (hip_joint_id < 0 || knee_joint_id < 0 ||
        hip_motor_id < 0 || knee_motor_id < 0)
    {
        std::cout << "Model needs hip_joint, knee_joint, hip_motor and knee_motor.\n";
        mj_deleteData(data);
        mj_deleteModel(model);
        rclcpp::shutdown();
        return 1;
    }

    int hip_qpos_address = model->jnt_qposadr[hip_joint_id];
    int hip_dof_address = model->jnt_dofadr[hip_joint_id];
    int knee_qpos_address = model->jnt_qposadr[knee_joint_id];
    int knee_dof_address = model->jnt_dofadr[knee_joint_id];

    double hip_torque_command = 0.0;
    double knee_torque_command = 0.0;

    auto joint_state_publisher = node->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", 1);

    auto hip_torque_subscription = node->create_subscription<std_msgs::msg::Float64>(
        "/hip/torque_cmd", 1,
        [&](const std_msgs::msg::Float64& message)
        {
            hip_torque_command = message.data;
        });

    auto knee_torque_subscription = node->create_subscription<std_msgs::msg::Float64>(
        "/knee/torque_cmd", 1,
        [&](const std_msgs::msg::Float64& message)
        {
            knee_torque_command = message.data;
        });

    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(2),
        [&]()
        {
            data->ctrl[hip_motor_id] = hip_torque_command;
            data->ctrl[knee_motor_id] = knee_torque_command;
            mj_step(model, data);

            sensor_msgs::msg::JointState message;
            message.header.stamp = node->now();
            message.name = {"hip_joint", "knee_joint"};
            message.position = {
                data->qpos[hip_qpos_address], data->qpos[knee_qpos_address]};
            message.velocity = {
                data->qvel[hip_dof_address], data->qvel[knee_dof_address]};
            joint_state_publisher->publish(message);
        });

    std::cout << "Torque-driven two-joint model loaded. Simulation dt: "
              << model->opt.timestep << " s.\n";

    rclcpp::spin(node);
    rclcpp::shutdown();

    mj_deleteData(data);
    mj_deleteModel(model);
    return 0;
}
