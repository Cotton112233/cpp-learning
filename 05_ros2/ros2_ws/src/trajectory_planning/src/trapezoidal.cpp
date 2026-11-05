#include "trajectory_planning/trapezoidal.hpp"

#include <cmath>
#include <limits>

namespace trajectory_planning
{

bool planTrapezoidal(double q0, double qf, double vmax, double amax, TrapezoidalPlan& plan)
{
    plan = {};
    if (!std::isfinite(q0) || !std::isfinite(qf) ||
        !std::isfinite(vmax) || !std::isfinite(amax) ||
        vmax <= 0.0 || amax <= 0.0)
    {
        return false;
    }

    plan.q0 = q0;
    plan.qf = qf;
    plan.amax = amax;
    double distance = std::abs(qf - q0);
    if (!std::isfinite(distance))
        return false;
    if (distance == 0.0)
        return true;

    plan.direction = qf > q0 ? 1.0 : -1.0;
    double time_to_vmax = vmax / amax;
    double critical_distance = vmax * time_to_vmax;

    // Accelerate and decelerate at amax; each ramp covers half its v*t.
    if (distance > critical_distance)
    {
        plan.peak_velocity = vmax;
        plan.acceleration_time = time_to_vmax;
        plan.cruise_time = (distance - critical_distance) / vmax;
    }
    else
    {
        // No cruise segment: D = amax * ta^2, v_peak = amax * ta.
        plan.acceleration_time = std::sqrt(distance / amax);
        plan.peak_velocity = amax * plan.acceleration_time;
        plan.cruise_time = 0.0;
    }
    plan.total_time = 2.0 * plan.acceleration_time + plan.cruise_time;

    return std::isfinite(plan.acceleration_time) &&
           std::isfinite(plan.cruise_time) &&
           std::isfinite(plan.peak_velocity) &&
           std::isfinite(plan.total_time) &&
           plan.acceleration_time > 0.0 && plan.total_time > 0.0;
}

TrajectoryState sampleTrapezoidal(double time, const TrapezoidalPlan& plan)
{
    double undefined = std::numeric_limits<double>::quiet_NaN();
    if (!std::isfinite(time))
        return {undefined, undefined, undefined};
    if (plan.q0 == plan.qf)
        return {plan.q0, 0.0, 0.0};
    if (time < 0.0)
        return {plan.q0, 0.0, 0.0};
    if (time > plan.total_time)
        return {plan.qf, 0.0, 0.0};
    if (time == 0.0)
        return {plan.q0, 0.0, undefined};
    if (time == plan.total_time)
        return {plan.qf, 0.0, undefined};

    double cruise_end = plan.acceleration_time + plan.cruise_time;
    double position;
    double velocity;
    double acceleration;

    if (time < plan.acceleration_time)
    {
        position = plan.q0 + plan.direction * 0.5 * plan.amax * time * time;
        velocity = plan.direction * plan.amax * time;
        acceleration = plan.direction * plan.amax;
    }
    else if (time < cruise_end)
    {
        double ramp_distance = 0.5 * plan.peak_velocity * plan.acceleration_time;
        position = plan.q0 + plan.direction *
            (ramp_distance + plan.peak_velocity * (time - plan.acceleration_time));
        velocity = plan.direction * plan.peak_velocity;
        acceleration = 0.0;
    }
    else
    {
        // Calculate backwards from qf for an exact endpoint and simple formulas.
        double remaining = plan.total_time - time;
        position = plan.qf - plan.direction * 0.5 * plan.amax * remaining * remaining;
        velocity = plan.direction * plan.amax * remaining;
        acceleration = -plan.direction * plan.amax;
    }

    if (time == plan.acceleration_time || time == cruise_end)
        acceleration = undefined;

    return {position, velocity, acceleration};
}

}
