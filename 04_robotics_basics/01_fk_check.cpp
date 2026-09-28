#include <cmath>
#include <iostream>

struct FKResult
{
    double knee_x;
    double knee_z;
    double foot_x;
    double foot_z;
};

FKResult forwardKinematics(double q1, double q2, double l1, double l2)
{
    FKResult result{};

    result.knee_x = -l1 * std::sin(q1);
    result.knee_z = -l1 * std::cos(q1);
    result.foot_x = result.knee_x - l2 * std::sin(q1 + q2);
    result.foot_z = result.knee_z - l2 * std::cos(q1 + q2);
    return result;
}

int main()
{
    const double pi = 3.1415926;
    const double q1 = -30.0 * pi / 180.0;
    const double q2 = 45.0 * pi / 180.0;
    const double l1 = 0.5;
    const double l2 = 0.5;

    const FKResult fk = forwardKinematics(q1, q2, l1, l2);

    std::cout << "knee_x: " << fk.knee_x << '\n'
              << "knee_z: " << fk.knee_z << '\n'
              << "foot_x: " << fk.foot_x << '\n'
              << "foot_z: " << fk.foot_z << '\n';

    return 0;
}