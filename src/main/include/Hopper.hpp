#pragma once

#include "Constants.hpp"
#include <frc/DigitalInput.h>

namespace Hopper
{
//public functions declared here

    void init();
    // index does not override shoot
    // returns whether or not it's indexing
    bool index(bool warn_if_shooting = true);
    void shoot(); // must call Hopper::stop() to stop shooting
    void stop();
}
