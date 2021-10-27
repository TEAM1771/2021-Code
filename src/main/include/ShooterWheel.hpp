#pragma once

#include "Constants.hpp"

namespace ShooterWheel
{
    void init();

    void   bangbang();
    void   stop();
    double get_speed();

    double get_temp();

    void maxSpeedForShooting(bool boolean);
} // namespace ShooterWheel
