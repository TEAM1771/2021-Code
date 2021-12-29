#pragma once

#include "Constants.hpp"

namespace Hood
{
    constexpr double TOLERANCE = 1;

    //Public Function Definitions
    void init();

    /// returns true if tolerance is met
    bool goToPosition(HOOD_POSITION position, double tolerance = TOLERANCE);

    /// returns true if tolerance is met
    bool visionTrack(double tolerance = TOLERANCE);

    /// used for tuning interpolation tables
    void manualPositionControl(double position);

    void   printAngle();
    double getAngle();
    double getCameraY();
} // namespace Hood
