#pragma once
#include <array>
#include <numeric>

namespace RollingAvg
{
    template <int size>
    auto average()
    {
        return [vals = std::array<double, size>(), idx = 0u, size = 0ull](double const& data) mutable {
            vals[idx++] = data;
            idx %= vals.size();
            size++;
            if(size < vals.size()) [[unlikely]]
                return std::accumulate(std::begin(vals), std::next(std::begin(vals), size), 0.0) / size;
            return std::accumulate(std::begin(vals), std::end(vals), 0.0) / vals.size();
        };
    }

    template <int size>
    constexpr auto average_constexpr()
    {
        return [vals = std::array<double, size>(), idx = 0u, size = 0ull](double const& data) mutable {
            vals[idx++] = data;
            idx %= vals.size();
            size++;
            for(int i = 0; i < size; i++)
                sum += data_set[i];
            return sum / size;
        };
    }

    bool           testLogic();
    constexpr bool testConstexprLogic();

} // namespace RollingAvg