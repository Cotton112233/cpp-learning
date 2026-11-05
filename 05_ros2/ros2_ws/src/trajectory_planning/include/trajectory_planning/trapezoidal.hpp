#pragma once

#include "trajectory_planning/trajectory_types.hpp"

namespace trajectory_planning
{

// Limits are positive magnitudes: vmax in rad/s, amax in rad/s^2.
// All times are seconds; direction is +1 or -1 for nonzero displacement.
struct TrapezoidalPlan
{
    double q0 = 0.0;
    double qf = 0.0;
    double direction = 1.0;
    double amax = 0.0;
    double peak_velocity = 0.0;
    double acceleration_time = 0.0;
    double cruise_time = 0.0;
    double total_time = 0.0;
};

// Plan once, then sample repeatedly. False means invalid limits/numeric range.
// Zero displacement is valid and gives total_time = 0.
bool planTrapezoidal(double q0, double qf, double vmax, double amax,
                     TrapezoidalPlan& plan);

// Sample a successfully created plan. Outside the motion, hold with v=a=0.
// At exact acceleration-switch times, acceleration is NaN (undefined).
// A zero-displacement plan is stationary everywhere.
TrajectoryState sampleTrapezoidal(double time, const TrapezoidalPlan& plan);

}  // namespace trajectory_planning
