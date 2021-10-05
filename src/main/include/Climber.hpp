#pragma once

#include "Constants.hpp"
#include "PID_CANSparkMax.hpp"

namespace Climber
{
//public functions declared
    void init();

    void set(CLIMBER::POSITION position);

    void ButtonManager();

    void joystickControl(double);

    void printStatus();
}