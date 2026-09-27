#include "pd_controller.hpp"

PDController::PDController(double kp, double kd)
    : kp_(kp), kd_(kd)
{
}

double PDController::calculate(double target_angle,
                               double current_angle,
                               double current_velocity,
                               double target_velocity) const
{
    double torque = 0;
    torque = kp_ * (target_angle - current_angle) + kd_ * (target_velocity - current_velocity);
    return torque;
}
