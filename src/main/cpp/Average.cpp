#include "Average.hpp"

double Average::average(double input)
{
    if (iterator == 5)
    {
        iterator = 0;
    }
    if (sizeOfDataSet < 5)
    {
        sizeOfDataSet++;
    }
    dataSet[iterator] = input;
    iterator++;
    double sum = 0;
    for (int i = 0; i < sizeOfDataSet; i++)
    {
        sum += dataSet[i];
    }
    return sum / sizeOfDataSet;
}
/*
double Average::average(double input, int amountToAvgBy)
{
    if (dataSet.size() < amountToAvgBy)
    {
        dataSet.emplace(iterator, input);
        iterator = dataSet.end();
    }
    else
    {
        dataSet[*iterator - 1] = input;
        iterator = std::next(iterator);
    }

    double debug_iterator = *iterator;

    if (dataSet.size() > amountToAvgBy)
    {
        dataSet.clear();
        iterator = dataSet.begin();
    }
    else if (*iterator == static_cast<double>(amountToAvgBy))
    {
        iterator = dataSet.begin();
    }

    double sum = 0;

    for (double i : dataSet)
    {
        sum += i;
    }

    return sum / dataSet.size();
}
*/
