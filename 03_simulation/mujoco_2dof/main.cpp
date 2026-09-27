#include <mujoco/mujoco.h>

#include <ctime>
#include <fstream>
#include <iostream>
#include <cmath>
#include <algorithm>
#include "pd_controller.hpp"
#include "viewer.hpp"

int main()
{
    const char* model_path = "two_dof.xml";

    char error[1024] = {};
    mjModel* model = mj_loadXML(model_path, nullptr, error, sizeof(error));
    if (model == nullptr)
    {
        std::cout << "Failed to load model: " << error << '\n';
        return 1;
    }

    mjData* data = mj_makeData(model);

    Viewer viewer(model);
    if (!viewer.isReady())
    {
        mj_deleteData(data);
        mj_deleteModel(model);
        return 1;
    }

    const int hip_joint_id = mj_name2id(model, mjOBJ_JOINT, "hip_joint");
    const int knee_joint_id = mj_name2id(model, mjOBJ_JOINT, "knee_joint");
    const int hip_motor_id = mj_name2id(model, mjOBJ_ACTUATOR, "hip_motor");
    const int knee_motor_id = mj_name2id(model, mjOBJ_ACTUATOR, "knee_motor");

    const int hip_qpos_address = model->jnt_qposadr[hip_joint_id];
    const int hip_dof_address = model->jnt_dofadr[hip_joint_id];
    const int knee_qpos_address = model->jnt_qposadr[knee_joint_id];
    const int knee_dof_address = model->jnt_dofadr[knee_joint_id];

    PDController hip_controller(50.0, 5.0);
    PDController knee_controller(50.0, 5.0);

    double hip_target_angle = 0.0;
    double hip_target_velocity = 0.0;
    double knee_target_angle = 0.0;
    double knee_target_velocity = 0.0;
    double hip_torque = 0.0;
    double knee_torque = 0.0;

    const std::time_t now = std::time(nullptr);
    const std::tm* local_time = std::localtime(&now);
    char filename[100] = {};
    std::strftime(filename, sizeof(filename),
                  "build/control_log_%Y%m%d_%H%M%S.csv", local_time);
    std::ofstream log_file(filename);
    log_file << "time,hip_target_angle,hip_actual_angle,hip_target_velocity,hip_velocity,hip_torque,"
                "knee_target_angle,knee_actual_angle,knee_target_velocity,knee_velocity,knee_torque\n";

    int step = 0;
    while (viewer.isOpen())
    {
        const double frame_end_time = data->time + 1.0 / 60.0;
        while (data->time < frame_end_time)
        {
            const double hip_angle = data->qpos[hip_qpos_address];
            const double hip_velocity = data->qvel[hip_dof_address];
            const double knee_angle = data->qpos[knee_qpos_address];
            const double knee_velocity = data->qvel[knee_dof_address];
            
            hip_target_angle = 0.5 * std::sin(data->time);
            knee_target_angle = 1.5 * std::cos(data->time);
            hip_target_velocity = 0.5 * std::cos(data->time);
            knee_target_velocity = -1.5 * std::sin(data->time);

            hip_torque = hip_controller.calculate(hip_target_angle, hip_angle, hip_velocity, hip_target_velocity);

            knee_torque = knee_controller.calculate(knee_target_angle, knee_angle, knee_velocity, knee_target_velocity);

            hip_torque = std::clamp(hip_torque, -30.0, 30.0);
            knee_torque = std::clamp(knee_torque, -30.0, 30.0);

            data->ctrl[hip_motor_id] = hip_torque;
            data->ctrl[knee_motor_id] = knee_torque;

            log_file << data->time << ','
                     << hip_target_angle << ',' << hip_angle << ','
                     << hip_target_velocity << ',' << hip_velocity << ',' << hip_torque << ','
                     << knee_target_angle << ',' << knee_angle << ','
                     << knee_target_velocity << ',' << knee_velocity << ',' << knee_torque << '\n';

            if (step % 100 == 0)
            {
                std::cout << "time: " << data->time
                          << ", hip: " << hip_angle
                          << ", knee: " << knee_angle << '\n';
            }

            mj_step(model, data);
            ++step;
        }

        viewer.render(model, data);
    }

    mj_deleteData(data);
    mj_deleteModel(model);
    return 0;
}