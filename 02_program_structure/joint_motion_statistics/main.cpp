#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include "statistics.h"

int main()
{
    std::ifstream input_file("joint_motion.csv");
    if(!input_file)
    {
        std::cout << "fail to open file"<<'\n';
        return 1;
    }

    std::string line;
    std::getline(input_file,line);//jump the header
    std::vector<double> angles;
    std::vector<double> velocities;
    while(std::getline(input_file,line))
    {

        if (line.empty())
        {
            continue;
        }

        std::stringstream ss(line);

        std::string time_text;
        std::string angle_text;
        std::string velocity_text;
        
        std::getline(ss,time_text,',');//ss(line) let getline can read pieces of line,','means read until meet ','
        std::getline(ss,angle_text,',');
        std::getline(ss,velocity_text,',');

        double angle = std::stod(angle_text);
        double velocity = std::stod(velocity_text);

        angles.push_back(angle);
        velocities.push_back(velocity);

    }

    if (angles.empty())
    {
        std::cout << "No valid data found.\n";
        return 1;
    }

    double average_angle{calculate_average(angles)};
    double average_velocity{calculate_average(velocities)};

    std::cout << "Number of samples: " << angles.size() << '\n'
              << "Average angle: " << average_angle << '\n'
              << "Average velocity: " << average_velocity << '\n';

    return 0;
}