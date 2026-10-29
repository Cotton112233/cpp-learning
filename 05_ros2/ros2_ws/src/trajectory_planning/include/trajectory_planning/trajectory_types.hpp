#pragma once

namespace trajectory_planning
{

// Revolute joint units: rad, rad/s, rad/s^2.
struct TrajectoryState
{
    double position = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
};

// dt belongs to the sampler, not the continuous trajectory formula.
struct TrajectoryParameters
{
    double q0 = 0.0;
    double qf = 1.0;
    double duration = 4.0;  // seconds
};

}  // namespace trajectory_planning
