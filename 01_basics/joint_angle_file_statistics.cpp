#include <iostream>
#include <vector>
#include <fstream>

int main()
{
    std::vector<double> joint_angles;
    double angle = 0 , sum = 0 , angle_max = 0 , angle_min = 0;
    double upper_lim = 20 , lower_lim = 10;
    int unsafe_count = 0;

    std::ifstream input_file;
    input_file.open("joint_angle.txt");             //read the file,and seach file in terminal's current working directory
    if (!input_file)        //check it's work or not
    {
        std::cout <<"failed to open the file"<<'\n';
        return 1;
    }

    while(input_file >>angle)      //get the data from file to vector
    {
        joint_angles.push_back(angle);
    }

    if(joint_angles.empty())        //check is it empty?
    {
        std::cout<<"no data in here"<<'\n';
        return 1 ;
    }

    angle_max = angle_min = joint_angles[0];

    for(double stored_angle:joint_angles)
    {
        std::cout <<stored_angle <<'\n';        //show what's in vecotr,and sum all the element
        sum += stored_angle;

        if(stored_angle>angle_max)              //find out max and min angle number
        {
            angle_max = stored_angle ;
        }
        else
        {
            if(stored_angle < angle_min)
            {
                angle_min = stored_angle;
            }   
        }

        if(lower_lim > stored_angle || upper_lim < stored_angle)//find out how many unsafe angle
        {
            unsafe_count ++ ;
        }
    }

    std::cout   <<"number read :"<<joint_angles.size()<<'\n'
                <<"average angle :"<<sum/joint_angles.size()<<'\n'
                <<"maximum angle :"<<angle_max<<'\n'
                <<"minimum angle :"<<angle_min<<'\n'
                <<"unsafe angle :"<<unsafe_count<<'\n';
    return 0;
}