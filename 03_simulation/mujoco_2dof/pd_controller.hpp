#pragma once

class PDController
{
public:
    PDController(double kp, double kd);

    double calculate(double target_angle,
                     double current_angle,
                     double current_velocity,
                     double target_velocity) const;

private:
    double kp_;
    double kd_;
};
