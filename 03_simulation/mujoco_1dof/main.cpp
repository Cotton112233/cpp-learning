#include <mujoco/mujoco.h>
#include <iostream>
#include "pd_controller.hpp"
#include "viewer.hpp"

int main()
{
    const char* model_path = "one_dof.xml";
    double torque_command = 0.0;

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

    const int joint_id = mj_name2id(model, mjOBJ_JOINT, "hinge_joint");
    const int actuator_id = mj_name2id(model, mjOBJ_ACTUATOR, "torque_motor");
    const int qpos_address = model->jnt_qposadr[joint_id];
    const int dof_address = model->jnt_dofadr[joint_id];

    const double target_angle = 0.5;
    PDController controller(50.0, 10.0);

    int step = 0;
    while (viewer.isOpen())
    {
        const double frame_end_time = data->time + 1.0 / 60.0;

        while (data->time < frame_end_time)
        {
            torque_command = controller.calculate(target_angle, data->qpos[qpos_address], data->qvel[dof_address]);

            data->ctrl[actuator_id] = torque_command;
            mj_step(model, data);

            if (step % 100 == 0)
            {
                std::cout << "time: " << data->time
                          << ", angle: " << data->qpos[qpos_address]
                          << ", velocity: " << data->qvel[dof_address]
                          << ", torque: " << torque_command << '\n';
            }

            ++step;
        }

        viewer.render(model, data);
    }

    mj_deleteData(data);
    mj_deleteModel(model);
    return 0;
}
