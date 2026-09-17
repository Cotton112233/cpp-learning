#include "joint_replay.hpp"

#include <chrono>
#include <iostream>
#include <thread>

void replaySamples(const std::vector<JointSample>& samples)
{

    for(const JointSample& sample : samples)
    {
        std::cout<<"time:"<<sample.time<<",angle:"<<sample.angle<<",velocity:"<<sample.velocity<<"\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return;
}
