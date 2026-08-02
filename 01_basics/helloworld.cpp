#include <iostream>

void setToZero(int& value)
{
    value = 0;
}

int main()
{
    int number = 10;

    // 引用：number 的另一个名字
    int& reference = number;

    // 指针：保存 number 的地址
    int* pointer = &number;

    reference = 20;  // 修改 number
    std::cout << number << '\n';  // 20

    *pointer = 30;   // 通过地址修改 number
    std::cout << number << '\n';  // 30

    setToZero(number);             // 引用参数修改 number
    std::cout << number << '\n';  // 0
}
