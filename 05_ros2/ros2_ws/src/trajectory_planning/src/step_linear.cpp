#include "trajectory_planning/step_linear.hpp"

#include <cmath>
#include <limits>

namespace trajectory_planning
{
namespace
{

TrajectoryState undefinedDerivatives(double position)
{
    double undefined = std::numeric_limits<double>::quiet_NaN();
    return {position, undefined, undefined};
}

bool validInputs(double time, const TrajectoryParameters& parameters)
{
    return std::isfinite(time) && std::isfinite(parameters.q0) &&
           std::isfinite(parameters.qf) && std::isfinite(parameters.duration) &&
           parameters.duration > 0.0;
}

}  // namespace

TrajectoryState stepTrajectory(double time, const TrajectoryParameters& parameters)
{
    if (!validInputs(time, parameters))
    {
        return undefinedDerivatives(std::numeric_limits<double>::quiet_NaN());
    }

    if (parameters.q0 == parameters.qf)
    {
        return {parameters.q0, 0.0, 0.0};
    }
    if (time < 0.0)
    {
        return {parameters.q0, 0.0, 0.0};
    }
    if (time == 0.0)
    {
        return undefinedDerivatives(parameters.qf);
    }
    return {parameters.qf, 0.0, 0.0};
}

TrajectoryState linearTrajectory(double time, const TrajectoryParameters& parameters)
{
    if (!validInputs(time, parameters))
    {
        return undefinedDerivatives(std::numeric_limits<double>::quiet_NaN());
    }

    double displacement = parameters.qf - parameters.q0;
    double velocity = displacement / parameters.duration;
    if (!std::isfinite(displacement) || !std::isfinite(velocity))
    {
        return undefinedDerivatives(std::numeric_limits<double>::quiet_NaN());
    }

    if (parameters.q0 == parameters.qf)
    {
        return {parameters.q0, 0.0, 0.0};
    }
    if (time < 0.0)
    {
        return {parameters.q0, 0.0, 0.0};
    }
    if (time > parameters.duration)
    {
        return {parameters.qf, 0.0, 0.0};
    }
    if (time == 0.0)
    {
        return undefinedDerivatives(parameters.q0);
    }
    if (time == parameters.duration)
    {
        return undefinedDerivatives(parameters.qf);
    }

    // Core formulas: q(t) = q0 + (qf - q0) * t / T, q_dot = (qf - q0) / T.
    double position = parameters.q0 + displacement * (time / parameters.duration);
    return {position, velocity, 0.0};
}

}  // namespace trajectory_planning
