#pragma once

namespace trajectory_planning
{

struct FKResult
{
    double knee_x;
    double knee_z;
    double foot_x;
    double foot_z;
};

struct IKResult
{
    bool success = false;
    double hip = 0.0;
    double knee = 0.0;
};

// Same convention as 04_robotics_basics: zero points down -z, axes along +y.
// Foot coordinates are relative to the fixed hip origin, in metres.
// q2 is relative to the thigh. Angles are radians.
FKResult forwardKinematics(double hip, double knee, double l1 = 0.5, double l2 = 0.5);

// Tries both knee branches; selects a valid candidate closest to zero posture.
// Both joint limits match two_dof.xml: [-1.57, 1.57] rad.
// If success is false, the returned angles must not be used.
IKResult inverseKinematics(double x, double z, double l1 = 0.5, double l2 = 0.5);

}  // namespace trajectory_planning
