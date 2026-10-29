#pragma once

#include "trajectory_planning/trajectory_types.hpp"

namespace trajectory_planning
{

// Both algorithms use the same time + parameters -> state interface.
// Invalid inputs return NaN in all fields. duration must be finite and > 0.
//
// Step: hold q0 for time < 0, switch to qf at time == 0, then hold qf.
// At a nonzero jump, velocity and acceleration at time == 0 are NaN.
// duration sets the CLI observation window; it does not smooth the jump.
TrajectoryState stepTrajectory(double time, const TrajectoryParameters& parameters);

// Linear: hold q0 before 0, interpolate during (0, duration), hold qf afterwards.
// With a nonzero displacement, derivatives at 0 and duration are undefined
// because the constant-velocity segment joins stationary segments: return NaN.
// With q0 == qf, both algorithms return a stationary state, including endpoints.
TrajectoryState linearTrajectory(double time, const TrajectoryParameters& parameters);

}  // namespace trajectory_planning
