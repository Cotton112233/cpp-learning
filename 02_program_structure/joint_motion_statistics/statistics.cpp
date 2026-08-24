#include "statistics.h"
#include <vector>

double calculate_average(const std::vector<double>& values)
{
    if(values.empty())
    {
        return 0.0;
    }

    double sum(0.0);

    for(double value : values)
    {
        sum += value;
    }

    return sum / static_cast<double>(values.size());//
}