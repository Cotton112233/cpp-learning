#include "trajectory_planning/polynomial.hpp"

#include <cmath>
#include <limits>

namespace trajectory_planning
{
namespace
{

double undefined()
{
    return std::numeric_limits<double>::quiet_NaN();
}

bool validInputs(double time, const TrajectoryParameters& parameters)
{
    return std::isfinite(time) && std::isfinite(parameters.q0) &&
           std::isfinite(parameters.qf) && std::isfinite(parameters.duration) &&
           parameters.duration > 0.0;
}

}  // namespace

TrajectoryState cubicTrajectory(double time, const TrajectoryParameters& parameters)
{
    if (!validInputs(time, parameters))
        return {undefined(), undefined(), undefined()};
    if (parameters.q0 == parameters.qf)
        return {parameters.q0, 0.0, 0.0};
    if (time < 0.0)
        return {parameters.q0, 0.0, 0.0};
    if (time > parameters.duration)
        return {parameters.qf, 0.0, 0.0};
    if (time == 0.0)
        return {parameters.q0, 0.0, undefined()};
    if (time == parameters.duration)
        return {parameters.qf, 0.0, undefined()};

    double T = parameters.duration;
    double s = time / T;  // Normalized progress: 0 < s < 1.
    double delta_q = parameters.qf - parameters.q0;

    double position = parameters.q0 + delta_q * (3.0 * s * s - 2.0 * s * s * s);
    double velocity = delta_q / T * (6.0 * s - 6.0 * s * s);
    double acceleration = delta_q / (T * T) * (6.0 - 12.0 * s);

    return {position, velocity, acceleration};
}

TrajectoryState quinticTrajectory(double time, const TrajectoryParameters& parameters)
{
    if (!validInputs(time, parameters))
        return {undefined(), undefined(), undefined()};
    if (parameters.q0 == parameters.qf)
        return {parameters.q0, 0.0, 0.0};
    if (time < 0.0)
        return {parameters.q0, 0.0, 0.0};
    if (time > parameters.duration)
        return {parameters.qf, 0.0, 0.0};
    if (time == 0.0)
        return {parameters.q0, 0.0, 0.0};
    if (time == parameters.duration)
        return {parameters.qf, 0.0, 0.0};

    double T = parameters.duration;
    double s = time / T;
    double delta_q = parameters.qf - parameters.q0;

    double s2 = s * s;
    double s3 = s2 * s;
    double s4 = s3 * s;
    double s5 = s4 * s;
    double position = parameters.q0 + delta_q * (10.0 * s3 - 15.0 * s4 + 6.0 * s5);
    double velocity = delta_q / T * (30.0 * s2 - 60.0 * s3 + 30.0 * s4);
    double acceleration = delta_q / (T * T) * (60.0 * s - 180.0 * s2 + 120.0 * s3);

    return {position, velocity, acceleration};
}

}
