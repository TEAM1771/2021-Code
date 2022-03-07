#include "Average.hpp"
#include <iostream>

constexpr bool RollingAvg::testConstexprLogic()
{
    auto avg = average_constexpr<3>();
    for(int i = 0; i < 10; i++)
    {
        double observed_value = avg(i);
        double expected_value = (i == 0) ? 0 : (i == 1) ? 0.5
                                                        : (i - 1);
        if(observed_value != expected_value)
            return false;
    }
    return true;
}

static_assert(RollingAvg::testConstexprLogic());

bool RollingAvg::testLogic()
{
    auto avg = average<3>();
    std::cout << "\n\nTesting RollingAverage (non constexpr version) logic!\n\n";
    for(int i = 0; i < 10; i++)
    {
        double observed_value = avg(i);
        double expected_value = (i == 0) ? 0 : (i == 1) ? 0.5
                                                        : (i - 1);
        std::cout << "Iterator: " << i << ", Observed: " << observed_value << ", Expected: " << expected_value << '\n';
        if(observed_value != expected_value)
        {
            return false;
            std::cout << "Error while testing: Expected Value: " << expected_value << " , Observed: " << observed_value << ".\n";
            // std::cout << "Current value in test_data_set[0] = " << test_data_set[0] << ", [1] = " << test_data_set[1] << ", [2] = " << test_data_set[2] << ".\n";
            // std::cout << "Current value of test_iterator = " << test_iterator << ", size_of_test_data_set = " << size_of_test_data_set << ".\n";
        }
    }
    return true;
}