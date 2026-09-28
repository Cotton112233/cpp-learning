#include <cmath>
#include <iostream>

int main()
{
    const double pi = 3.1415926;
    double q1 = -30.0 * pi / 180.0;
    double q2 = 45.0 * pi / 180.0;
    double q1_dot = 0.2;
    double q2_dot = -0.1;
    double l1 = 0.5;
    double l2 = 0.5;
    double vx = 0;
    double vz = 0;

    double J[2][2] = {
    { -l1 * std::cos(q1) - l2 * std::cos(q1 + q2), - l2 * std::cos(q1 + q2) },
    { l1 * std::sin(q1) + l2 * std::sin(q1 + q2), l2 * std::sin(q1 + q2) }
    };

    vx = J[0][0] * q1_dot + J[0][1] * q2_dot;
    vz = J[1][0] * q1_dot + J[1][1] * q2_dot;

    std::cout << J[0][0] << " " << J[0][1] << '\n'
              << J[1][0] << " " << J[1][1] << '\n'
              << "vx= " << vx << "m/s" << '\n'
              << "vz= " << vz << "m/s" << '\n';

    return 0;
}
