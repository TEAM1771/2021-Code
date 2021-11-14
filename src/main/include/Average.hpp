#pragma once
#include <iostream>
#include <vector>

template <int size>
class Average
{   //Class since we might use this multiple times & it needs a static dataSet
    //    std::vector<double> dataSet;
    //    std::vector<double>::iterator iterator = dataSet.end();
    double dataSet[size];
    int    iterator      = 0;
    int    sizeOfDataSet = 0;

public:
    constexpr double operator()(double input)
    {
        if(iterator == size)
        {
            iterator = 0;
        }
        if(sizeOfDataSet < size)
        {
            sizeOfDataSet++;
        }
        dataSet[iterator] = input;
        iterator++;
        double sum = 0;
        for(int i = 0; i < sizeOfDataSet; i++)
        {
            sum += dataSet[i];
        }
        return sum / sizeOfDataSet;
    }
};