#include "Intake.hpp"
#include <iostream>

//private variables
    frc::Solenoid intakeair { INTAKE::PCM_PORT };
    bool          intakeDeployed = false;

    rev::CANSparkMax wheels { INTAKE::PORT, rev::CANSparkMaxLowLevel::MotorType::kBrushless };

//public function definitions
void Intake::init()
{
    wheels.SetIdleMode(INTAKE::IDLE_MODE);
}

void Intake::drive(INTAKE::DIRECTION mode)
{
    switch(mode)
    {
    case INTAKE::DIRECTION::IN:
        wheels.Set(INTAKE::IN_SPEED);
        break;
    case INTAKE::DIRECTION::OFF:
        wheels.Set(0);
        break;
    case INTAKE::DIRECTION::OUT:
        wheels.Set(INTAKE::OUT_SPEED);
        break;
    default:
        std::cerr << "Invalad Intake Direction\n";
    }
}

void Intake::deploy(bool val)
{
    intakeair.Set(val);
    intakeDeployed = val;
}

bool Intake::isIntakeDown()
{
    return intakeDeployed;
}
