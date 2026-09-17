#pragma once

#include <vector>

#include "joint_sample.hpp"

class JointStatistics
{
public:
    void calculate(const std::vector<JointSample>& samples);

    double averageAngle() const;
    double minimumAngle() const;
    double maximumAngle() const;
    double averageVelocity() const;
    double minimumVelocity() const;
    double maximumVelocity() const;

private:
    double average_angle_ = 0.0;
    double minimum_angle_ = 0.0;
    double maximum_angle_ = 0.0;
    double average_velocity_ = 0.0;
    double minimum_velocity_ = 0.0;
    double maximum_velocity_ = 0.0;
};
