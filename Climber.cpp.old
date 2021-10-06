#include "Climber.hpp"
#include "Constants.hpp"

Climber::Climber()
{
    climber_1.RestoreFactoryDefaults();
    climber_2.RestoreFactoryDefaults();

    climber_1.SetIdleMode(CLIMBER::IDLE_MODE);
    climber_2.SetIdleMode(CLIMBER::IDLE_MODE);

    climber_1.SetP(CLIMBER::P);
    climber_1.SetI(CLIMBER::I);
    climber_1.SetD(CLIMBER::D);
    climber_1.SetOutputRange(-CLIMBER::MAX_OUTPUT, CLIMBER::MAX_OUTPUT);
    climber_2.SetPositionRange(CLIMBER::POSITION::ZERO, CLIMBER::POSITION::UP);
    climber_1.SetTarget(CLIMBER::POSITION::ZERO);

    climber_2.SetP(CLIMBER::P);
    climber_2.SetI(CLIMBER::I);
    climber_2.SetD(CLIMBER::D);
    climber_2.SetOutputRange(-CLIMBER::MAX_OUTPUT, CLIMBER::MAX_OUTPUT);
    climber_2.SetPositionRange(-CLIMBER::POSITION::UP, CLIMBER::POSITION::ZERO);
    climber_2.SetTarget(CLIMBER::POSITION::ZERO);
}

void Climber::set(CLIMBER::POSITION position)
{
    climber_1.SetTarget(position, rev::ControlType::kPosition);
    climber_2.SetTarget(-position, rev::ControlType::kPosition);
}

void Climber::joystickControl(double val)
{
    climber_1.Set(val);
    climber_2.Set(-val);
    printStatus();
}

void Climber::printStatus()
{
    std::cout << "Climber 1: " << climber_1.encoder.GetPosition() << std::endl;
    std::cout << "Climber 2: " << climber_2.encoder.GetPosition() << std::endl;
}

void Climber::ButtonManager()
{
    static bool hasBeenPressed = false;
    if(BUTTON::CLIMBER::RAISE && BUTTON::oStick.GetThrottle() < 0)
    {
        hasBeenPressed = true;
        set(CLIMBER::POSITION::UP);
    }
    else if(hasBeenPressed)
        set(CLIMBER::POSITION::DOWN);
}