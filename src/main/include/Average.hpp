#pragma once
#include <array>
#include <numeric>

template <int max_size>
auto RollingAverage = [data_set = std::array<double, max_size>(), i = 0, current_size = 0](double input) mutable -> double {
    if(i == max_size)
        i = 0;
    if(current_size < max_size)
        current_size++;
    data_set[i++] = input;
    return std::accumulate(data_set.begin(), data_set.end(), 0.0) / current_size;
};

template <int max_size>
constexpr auto RollingAverageConstexpr = [data_set = std::array<double, max_size>(), i = 0, current_size = 0](double input) mutable -> double {
    if(i == max_size)
        i = 0;
    if(current_size < max_size)
        current_size++;
    data_set[i++] = input;
    double sum    = 0.00;
    for(int i = 0; i < current_size; i++)
        sum += data_set[i];
    return sum / current_size;
};

namespace RollingAvg
{
    constexpr bool testLogic();
}