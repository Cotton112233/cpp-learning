#pragma once

#include "trajectory_planning/trajectory_types.hpp"

namespace trajectory_planning
{

// Same interface as stepTrajectory and linearTrajectory; no ROS dependencies.
// Invalid inputs return NaN in all fields. Outside [0,T], hold position with v=a=0.
// Cubic: zero endpoint velocity. Under the stationary extension, endpoint
// acceleration is undefined, so at exactly 0 and T it is marked NaN.
// Quintic: zero endpoint velocity and acceleration, including exactly 0 and T.
// If q0 == qf, return a stationary state everywhere.
TrajectoryState cubicTrajectory(double time, const TrajectoryParameters& parameters);
TrajectoryState quinticTrajectory(double time, const TrajectoryParameters& parameters);

}  // namespace trajectory_planning
