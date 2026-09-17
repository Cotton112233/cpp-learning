#include "joint_statistics.hpp"

void JointStatistics::calculate(const std::vector<JointSample>& samples)
{
    if (samples.empty())
    {
        return;
    }
    double angle_sum = 0;
    double velocity_sum = 0;
    minimum_angle_ = samples[0].angle;
    maximum_angle_ = samples[0].angle;
    minimum_velocity_ = samples[0].velocity;
    maximum_velocity_ = samples[0].velocity;

    for(const JointSample& sample : samples)
    {
        angle_sum += sample.angle;
        velocity_sum += sample.velocity;

        if(sample.angle < minimum_angle_)
        {
            minimum_angle_ = sample.angle;
        }
        if(sample.angle > maximum_angle_)
        {
            maximum_angle_ = sample.angle;
        }

        if(sample.velocity < minimum_velocity_)
        {
            minimum_velocity_ = sample.velocity;
        }
        if(sample.velocity > maximum_velocity_)
        {
            maximum_velocity_ = sample.velocity;
        }

    }

    average_angle_ = angle_sum / static_cast<double>(samples.size());
    average_velocity_ = velocity_sum / static_cast<double>(samples.size());
}

double JointStatistics::averageAngle() const
{
    return average_angle_;
}

double JointStatistics::minimumAngle() const
{
    return minimum_angle_;
}

double JointStatistics::maximumAngle() const
{
    return maximum_angle_;
}

double JointStatistics::averageVelocity() const
{
    return average_velocity_;
}

double JointStatistics::minimumVelocity() const
{
    return minimum_velocity_;
}

double JointStatistics::maximumVelocity() const
{
    return maximum_velocity_;
}
