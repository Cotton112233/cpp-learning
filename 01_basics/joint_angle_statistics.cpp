#include <iostream>
#include <vector>

int main()
{
    std::vector<double> joint_angles;
    double angle_avg = 0;
    double sum = 0;
    double angle_max ;
    double angle_min ;
    const double lower_limit = 1;
    const double upper_limit = 3;
    bool lower_flag = false;
    bool upper_flag = false;
    int unsafe_angle_count = 0;

    for(int i = 0;i<5;i++)                  //store serveral joint angles
    {
        joint_angles.push_back(i);
    }
    
    for(double angle:joint_angles)          //print every angle and sum all the element
    {
        std::cout << angle << '\n' ;
        sum += angle;
    }

    angle_avg = sum/joint_angles.size();    //calculate and show the average
    std::cout << angle_avg <<'\n';

    angle_max = angle_min = joint_angles[0];
    for(double tmp_angle : joint_angles)    //find max and min in joint_angle
    {
        if(tmp_angle >= angle_max)
            angle_max = tmp_angle;

        if(tmp_angle <= angle_min)
            angle_min = tmp_angle;
    }
    std::cout <<"angle max = " << angle_max<<'\n'<<"angle min ="<<angle_min<<'\n';

    if(angle_max > upper_limit)
        upper_flag = true;
    if(angle_min < lower_limit)
        lower_flag = true;
    std::cout <<"touch the top = " << upper_flag<<'\n'<<"touch the bottom ="<<lower_flag<<'\n';

    for(double angle:joint_angles)
    {
        if(angle<lower_limit||angle>upper_limit)
            unsafe_angle_count ++;
    }
    std::cout << "unsafe angle count ="<<unsafe_angle_count<<'\n';

    return 0;
}
