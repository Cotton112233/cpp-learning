#include "trajectory_planning/planar_ik.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace trajectory_planning
{

FKResult forwardKinematics(double hip, double knee, double l1, double l2)
{
    FKResult result{};
    result.knee_x = -l1 * std::sin(hip);
    result.knee_z = -l1 * std::cos(hip);
    result.foot_x = result.knee_x - l2 * std::sin(hip + knee);
    result.foot_z = result.knee_z - l2 * std::cos(hip + knee);
    return result;
}

IKResult inverseKinematics(double x, double z, double l1, double l2)
{
    IKResult best;
    if (!std::isfinite(x) || !std::isfinite(z) ||
        !std::isfinite(l1) || !std::isfinite(l2) || l1 <= 0.0 || l2 <= 0.0)
    {
        return best;
    }

    double cos_q2 = (x * x + z * z - l1 * l1 - l2 * l2)
                    / (2.0 * l1 * l2);
    if (cos_q2 < -1.0 - 1e-10 || cos_q2 > 1.0 + 1e-10)
    {
        return best;
    }
    cos_q2 = std::clamp(cos_q2, -1.0, 1.0);

    double knee_magnitude = std::acos(cos_q2);
    double alpha = std::atan2(-x, -z);
    double best_cost = std::numeric_limits<double>::infinity();
    for (double q2 : {knee_magnitude, -knee_magnitude})
    {
        double delta = std::atan2(l2 * std::sin(q2),
                                  l1 + l2 * std::cos(q2));
        double q1 = alpha - delta;
        q1 = std::atan2(std::sin(q1), std::cos(q1));

        if (q1 < -1.57 || q1 > 1.57 || q2 < -1.57 || q2 > 1.57)
        {
            continue;
        }

        double cost = q1 * q1 + q2 * q2;
        if (cost < best_cost)
        {
            best = {true, q1, q2};
            best_cost = cost;
        }
    }
    return best;
}

}
