#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

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

    double angle_sum = 0.0;
    double velocity_sum = 0.0;

    double minimum_angle = angles[0];
    double maximum_angle = angles[0];

    double minimum_velocity = velocities[0];
    double maximum_velocity = velocities[0];

    for (std::size_t i = 0; i < angles.size(); ++i)
    {
        angle_sum += angles[i];
        velocity_sum += velocities[i];

        for (std::size_t i = 0; i < angles.size(); ++i)
        {
            angle_sum += angles[i];
            velocity_sum += velocities[i];

            if (angles[i] < minimum_angle)
            {
                minimum_angle = angles[i];
            }

            if (angles[i] > maximum_angle)
            {
                maximum_angle = angles[i];
            }

            if (velocities[i] < minimum_velocity)
            {
                minimum_velocity = velocities[i];
            }

            if (velocities[i] > maximum_velocity)
            {
                maximum_velocity = velocities[i];
            }
        }
    }

    double average_angle = angle_sum / static_cast<double>(angles.size());

    double average_velocity = velocity_sum / static_cast<double>(velocities.size());

    std::cout << "Number of samples: " << angles.size() << '\n'
              << "Average angle: " << average_angle << '\n'
              << "Minimum angle: " << minimum_angle << '\n'
              << "Maximum angle: " << maximum_angle << '\n'
              << "Average velocity: " << average_velocity << '\n'
              << "Minimum velocity: " << minimum_velocity << '\n'
              << "Maximum velocity: " << maximum_velocity << '\n';


    return 0;
}