#include <iostream>
#include <string>

int main()
{
    std::string joint = "hip";
    std::string side = "left";
    std::string joint_name = side + " " + joint;
    std::cout << joint_name <<'\n';
    std::cout << joint_name.size() << '\n';     //find the length

    std::cout << joint_name[0];
    std::cout << joint_name[1];
    std::cout << joint_name[2];
    std::cout << joint_name[3];
    std::cout << '\n';                          //access one charscter

    if(side == "left")                          //compare
    {
        std::cout<< "left side"<<'\n';
    }

    std::cin >> joint_name;          //stop at space,read one word
    std::cout << joint_name<< '\n'; //if input "ab cd",joint_name ="ab",and input buffer leave " cd\n"

    std::getline(std::cin,joint_name);//stop at enter,read whole line.//——>and auto get " cd\n"
    std::cout << joint_name<< '\n';

    std::cout << "test";                //with out '\n'
    std::getline(std::cin>>std::ws,joint_name);//std::ws remove whitespace and getline() read the full line 
    std::cout << joint_name<< '\n';

    return 0;
}