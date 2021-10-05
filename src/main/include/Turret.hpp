#pragma once

#include "Constants.hpp"
#include "LimeLight.hpp"
#include "PID_CANSparkMax.hpp"

namespace Turret
{
    struct visionState
{
    bool isTracking;
    bool readyToShoot;
};
//public function declarations
    void init();

    /// returns true if tolerance is met
    bool goToPosition(TURRET::POSITION position, double tolerance = 1);

    /// goes to position and then starts tracking, returns true if tolerance (in degrees) is met
    [[depricated]] visionState visionTrack_v1(TURRET::POSITION initPosition, double tolerance = 10);

    /// goes to position, then determines angle of target and goes to that angle
    visionState visionTrack(TURRET::POSITION initPosition, double tolerance = TURRET::TOLERANCE);

    /// used for tuning interpolation tables
    void manualPositionControl(double position);
}
