#include <charconv>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string_view>
#include <system_error>

#include "trajectory_planning/planar_ik.hpp"

namespace
{
bool number(std::string_view text, double& value)
{
    auto result = std::from_chars(text.data(), text.data() + text.size(), value);
    return result.ec == std::errc{} && result.ptr == text.data() + text.size()
           && std::isfinite(value);
}
}

int main(int argc, char* argv[])
{
    if (argc != 3)
    {
        std::cerr << "Usage: ik_cli X Z (metres relative to fixed hip origin)\n";
        return 1;
    }
    double x, z;
    if (!number(argv[1], x) || !number(argv[2], z))
    {
        std::cerr << "X and Z must be finite numbers.\n";
        return 1;
    }

    auto ik = trajectory_planning::inverseKinematics(x, z);
    if (!ik.success)
    {
        std::cerr << "IK failed: target outside geometric reach or joint limits.\n";
        return 2;
    }
    auto fk = trajectory_planning::forwardKinematics(ik.hip, ik.knee);
    std::cout << std::setprecision(std::numeric_limits<double>::max_digits10)
              << "hip,knee,foot_x,foot_z\n"
              << ik.hip << ',' << ik.knee << ',' << fk.foot_x << ',' << fk.foot_z << '\n';
    return 0;
}
