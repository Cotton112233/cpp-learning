#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "joint_replay.hpp"
#include "joint_sample.hpp"
#include "joint_statistics.hpp"

int main()
{
    std::ifstream input_file("joint_motion.csv");
    if (!input_file)
    {
        std::cout << "Failed to open file.\n";
        return 1;
    }

    std::string line;
    std::getline(input_file, line);

    std::vector<JointSample> samples;

    while (std::getline(input_file, line))
    {
        if (line.empty())
        {
            continue;
        }

        std::stringstream row(line);
        std::string time_text;
        std::string angle_text;
        std::string velocity_text;

        std::getline(row, time_text, ',');
        std::getline(row, angle_text, ',');
        std::getline(row, velocity_text, ',');

        JointSample sample{};
        sample.time = std::stod(time_text);
        sample.angle = std::stod(angle_text);
        sample.velocity = std::stod(velocity_text);

        samples.push_back(sample);
    }

    if (samples.empty())
    {
        std::cout << "No valid data found.\n";
        return 1;
    }

    JointStatistics statistics;
    statistics.calculate(samples);

    std::cout << "Number of samples: " << samples.size() << '\n'
              << "Average angle: " << statistics.averageAngle() << '\n'
              << "Minimum angle: " << statistics.minimumAngle() << '\n'
              << "Maximum angle: " << statistics.maximumAngle() << '\n'
              << "Average velocity: " << statistics.averageVelocity() << '\n'
              << "Minimum velocity: " << statistics.minimumVelocity() << '\n'
              << "Maximum velocity: " << statistics.maximumVelocity() << '\n';

    replaySamples(samples);

    return 0;
}
