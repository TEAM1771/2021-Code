#pragma once

#include "Constants.hpp"
#include <frc/DigitalInput.h>

class Hopper
{
    rev::CANSparkMax indexer { HOPPER::INDEXER::PORT, rev::CANSparkMaxLowLevel::MotorType::kBrushless };
    rev::CANSparkMax transport { HOPPER::TRANSPORT::PORT, rev::CANSparkMaxLowLevel::MotorType::kBrushless };

    rev::CANPIDController pidController = transport.GetPIDController();
    rev::CANEncoder       encoder       = transport.GetEncoder();

    frc::DigitalInput limitSwitch { HOPPER::LIMIT_SWITCH };

    int               numberOfBalls  = 3;
    double            targetDistance = HOPPER::TRANSPORT::DISTANCE;
    bool              isTransporting = false;
    std::atomic<bool> invalidStopFlag { false };

    void driveDistance();

public:
    Hopper();
    // index does not override shoot
    // returns whether or not it's indexing
    bool index(bool warn_if_shooting = true);
    void shoot(); // must call Hopper::stop() to stop shooting
    void stop();
};
