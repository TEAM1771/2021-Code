#pragma once

#include "Constants.hpp"

namespace Climber
{
    //Function Declarations
    void init();
    void set(CLIMBER_POSITION position);
    void buttonManager();
    void joystickControl(double);
    void printStatus();
} // namespace Climber