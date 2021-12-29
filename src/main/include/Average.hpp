#pragma once
#include <iostream>
#include <assert.h>

template <int size>
class Average
{
    double data_set[size];
    int iterator = 0;
    int size_of_data_set = 0;

public:
    Average()
    {
        assert(testLogic());
    }

    void clear()
    {
        int iterator = 0;
        int size_of_data_set = 0;
    }

    double operator()(double input)
    {
        if (iterator == size)
        {
            iterator = 0;
        }
        if (size_of_data_set < size)
        {
            size_of_data_set++;
        }
        data_set[iterator] = input;
        iterator++;
        double sum = 0;
        for (int i = 0; i < size_of_data_set; i++)
        {
            sum += data_set[i];
        }
        return sum / size_of_data_set;
    }

        bool testLogic() const
    {
        double test_data_set[3]{2234.69, 5648415, -.14};
        int test_size = 3;
        int test_iterator = 0;
        int size_of_test_data_set = 0;
        auto avg = [&test_data_set, &test_iterator, &size_of_test_data_set, &test_size](double input)
        {
            if (test_iterator >= test_size)
            {
                test_iterator = 0;
            }
            if (size_of_test_data_set < test_size)
            {
                size_of_test_data_set++;
            }
            test_data_set[test_iterator] = input;
            test_iterator++;
            double sum = 0;
            for (int i = 0; i < size_of_test_data_set; i++)
            {
                sum += test_data_set[i];
            }
            return sum / size_of_test_data_set;
        };

        double expected_value;
        for (int i = 0; i < 10; i++)
        {
            double observed_value = avg(i);
            if (i == 0)
                expected_value = 0;
            else if (i == 1)
                expected_value = 0.5;
            else
                expected_value = i - 1;

            if (observed_value != expected_value)
            {
                std::cout << "Error while testing Average logic: Expected Value: " << expected_value << " , Observed: " << observed_value << ".\n";
                std::cout << "Current value in test_data_set[0] = " << test_data_set[0] << ", [1] = " << test_data_set[1] << ", [2] = " << test_data_set[2] << ".\n";
                std::cout << "Current value of test_iterator = " << test_iterator << ", size_of_test_data_set = " << size_of_test_data_set << ".\n";
                return false;
            }
        }
        // if (avg(0) != 0){
        //     std::cout << +
        //     return false;
        // }
        // if (avg(1) != 0.5)
        //     return false;
        // if (avg(2) != 1)
        //     return false;
        // if (avg(3) != 2)
        //     return false;
        // if (avg(4) != 3)
        //     return false;
        // if (avg(5) != 4)
        //     return false;
        // if (avg(6) != 5)
        //     return false;
        // if (avg(7) != 6)
        //     return false;
        return true;
    }
};