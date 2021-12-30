#pragma once
#include <array>
#include <numeric>

namespace RollingAvg
{
    template <int max_size>
    auto avg = [data_set = std::array<double, max_size>(), i = 0, current_size = 0](double const& input) mutable -> double {
        if(i == max_size)
            i = 0;
        if(current_size < max_size)
            current_size++;
        data_set[i++] = input;
        return std::accumulate(data_set.begin(), data_set.end(), 0.0) / current_size;
    };

    template <int max_size>
    constexpr auto constexprAvg = [data_set = std::array<double, max_size>(), i = 0, current_size = 0](double const& input) mutable -> double {
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

    bool           testLogic();
    constexpr bool testConstexprLogic();

} // namespace RollingAvg