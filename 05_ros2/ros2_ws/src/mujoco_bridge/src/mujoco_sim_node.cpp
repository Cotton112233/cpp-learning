#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include <mujoco/mujoco.h>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "pd_controller.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mujoco_sim_node");

    std::string model_path =
        ament_index_cpp::get_package_share_directory("mujoco_bridge") +
        "/models/one_dof.xml";

    char error[1024] = {};
    mjModel* model = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    if (model == nullptr)
    {
        std::cout << "Failed to load model: " << error << '\n';
        rclcpp::shutdown();
        return 1;
    }

    mjData* data = mj_makeData(model);

    int joint_id = mj_name2id(model, mjOBJ_JOINT, "hinge_joint");
    int actuator_id = mj_name2id(model, mjOBJ_ACTUATOR, "torque_motor");
    if (joint_id < 0 || actuator_id < 0)
    {
        std::cout << "Model needs hinge_joint and torque_motor.\n";
        mj_deleteData(data);
        mj_deleteModel(model);
        rclcpp::shutdown();
        return 1;
    }

    int qpos_address = model->jnt_qposadr[joint_id];
    int dof_address = model->jnt_dofadr[joint_id];

    double target_angle = 0.0;
    PDController controller(500.0, 30.0);

    auto current_angle_publisher = node->create_publisher<std_msgs::msg::Float64>(
        "/hip/current_angle", 10);

    auto target_angle_subscription = node->create_subscription<std_msgs::msg::Float64>(
        "/hip/target_angle", 10,
        [&](const std_msgs::msg::Float64& message)
        {
            target_angle = message.data;
        });

    int step = 0;
    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(2),
        [&]()
        {
            double current_angle = data->qpos[qpos_address];
            double current_velocity = data->qvel[dof_address];
            double torque_command = 0.0;

            torque_command = controller.calculate(target_angle,current_angle,current_velocity,0.0);

            torque_command = std::clamp(torque_command, -100.0, 100.0);
            data->ctrl[actuator_id] = torque_command;
            mj_step(model, data);

            ++step;
            if (step % 50 == 0)
            {
                std_msgs::msg::Float64 message;
                message.data = data->qpos[qpos_address];
                current_angle_publisher->publish(message);
            }
        });

    std::cout << "Model loaded. Simulation dt: " << model->opt.timestep
              << " s; publishing /hip/current_angle.\n";

    rclcpp::spin(node);
    rclcpp::shutdown();

    mj_deleteData(data);
    mj_deleteModel(model);
    return 0;
}
