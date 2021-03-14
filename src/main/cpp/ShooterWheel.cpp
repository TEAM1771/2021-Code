#include "ShooterWheel.hpp"

ShooterWheel::ShooterWheel()
{
    shooter_1.RestoreFactoryDefaults();
    shooter_2.RestoreFactoryDefaults();

    shooter_2.SetIdleMode(SHOOTER_WHEEL::IDLE_MODE);
    shooter_1.SetIdleMode(SHOOTER_WHEEL::IDLE_MODE);
}

void ShooterWheel::bangbang() // origional code with commented code removed
{   
    return; 
    if(abs(shooter_encoder.GetVelocity() > SHOOTER_WHEEL::SHOOTING_RPM - 1200))
        shooter_1.SetOpenLoopRampRate(0);
    else
        shooter_1.SetOpenLoopRampRate(6);

    if(abs(shooter_encoder.GetVelocity() > SHOOTER_WHEEL::SHOOTING_RPM - 2000))
        shooter_1.SetOpenLoopRampRate(0);
    else
        shooter_1.SetOpenLoopRampRate(6);

    if(abs(shooter2_encoder.GetVelocity() > SHOOTER_WHEEL::SHOOTING_RPM - 2000))
        shooter_2.SetOpenLoopRampRate(0);
    else
        shooter_2.SetOpenLoopRampRate(6);


    if((abs(shooter_encoder.GetVelocity()) < SHOOTER_WHEEL::SHOOTING_RPM - 250))
        shooter_1.Set(-1);
    else
        shooter_1.Set(0);

    if((abs(shooter2_encoder.GetVelocity()) < SHOOTER_WHEEL::SHOOTING_RPM - 250))
        shooter_2.Set(1);
    else
        shooter_2.Set(0);
}