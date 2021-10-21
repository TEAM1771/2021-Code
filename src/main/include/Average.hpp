#pragma once
#include <iostream>
#include <vector>

class Average
{ //Class since we might use this multiple times & it needs a static dataSet
//    std::vector<double> dataSet;
//    std::vector<double>::iterator iterator = dataSet.end();
      double dataSet[5];
      int iterator = 0;
      int sizeOfDataSet = 0;

public:
    double operator() (double input);
};