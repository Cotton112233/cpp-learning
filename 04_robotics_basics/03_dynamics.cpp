#include <mujoco/mujoco.h>
#include <iostream>

int main()
{
    char error[1024] = {};
    mjModel* model = mj_loadXML("two_dof_dynamics.xml", nullptr, error, sizeof(error));
    if (model == nullptr)
    {
        std::cout << "Failed to load model: " << error << '\n';
        return 1;
    }

    mjData* data = mj_makeData(model);

    int hip_joint_id = mj_name2id(model, mjOBJ_JOINT, "hip_joint");
    int knee_joint_id = mj_name2id(model, mjOBJ_JOINT, "knee_joint");
    int hip_motor_id = mj_name2id(model, mjOBJ_ACTUATOR, "hip_motor");
    int knee_motor_id = mj_name2id(model, mjOBJ_ACTUATOR, "knee_motor");

    int hip_qpos_address = model->jnt_qposadr[hip_joint_id];
    int hip_dof_address = model->jnt_dofadr[hip_joint_id];
    int knee_qpos_address = model->jnt_qposadr[knee_joint_id];
    int knee_dof_address = model->jnt_dofadr[knee_joint_id];

    data->qpos[hip_qpos_address] = 0.4;
    data->qpos[knee_qpos_address] = 0.5;

    data->qvel[hip_dof_address] = 0.0;
    data->qvel[knee_dof_address] = 0.0;

    data->ctrl[hip_motor_id] = 1.0;
    data->ctrl[knee_motor_id] = 0.0;

    std::cout << "time(s), hip_angle(rad), hip_velocity(rad/s), "
              << "knee_angle(rad), knee_velocity(rad/s)\n";

    int step = 0;
    while (data->time < 3.0)
    {
        mj_step(model, data);

        if (step % 50 == 0)
        {
            std::cout << data->time << ", "
                      << data->qpos[hip_qpos_address] << ", "
                      << data->qvel[hip_dof_address] << ", "
                      << data->qpos[knee_qpos_address] << ", "
                      << data->qvel[knee_dof_address] << '\n';
        }

        ++step;
    }

    mj_deleteData(data);
    mj_deleteModel(model);
    return 0;
}
