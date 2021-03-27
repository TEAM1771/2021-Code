#pragma once

#include "Constants.hpp"

class ShooterWheel
{
    rev::CANSparkMax shooter_1 { SHOOTER_WHEEL::PORT_1, rev::CANSparkMaxLowLevel::MotorType::kBrushless };
    rev::CANEncoder shooter_encoder  = shooter_1.GetEncoder();
public:
    ShooterWheel();

    void bangbang();
};
