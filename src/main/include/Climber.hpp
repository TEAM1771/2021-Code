#pragma once

#include "Constants.hpp"
#include "PID_CANSparkMax.hpp"

class Climber
{
    PID_CANSparkMax climber_1 { CLIMBER::PORT_1, rev::CANSparkMaxLowLevel::MotorType::kBrushless };
    PID_CANSparkMax climber_2 { CLIMBER::PORT_2, rev::CANSparkMaxLowLevel::MotorType::kBrushless };

public:
    Climber();

    void set(CLIMBER::POSITION position);

    void ButtonManager();

    void joystickControl(double);

    void printStatus();
};