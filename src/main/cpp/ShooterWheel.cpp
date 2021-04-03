#include "ShooterWheel.hpp"

ShooterWheel::ShooterWheel()
{
    shooter_1.RestoreFactoryDefaults();
    shooter_1.SetIdleMode(SHOOTER_WHEEL::IDLE_MODE);
    shooter_1.SetP(SHOOTER_WHEEL::P);
    shooter_1.SetI(SHOOTER_WHEEL::I);
    shooter_1.SetD(SHOOTER_WHEEL::D);
    shooter_1.SetDFilter(SHOOTER_WHEEL::F);
    shooter_1.SetFF(-SHOOTER_WHEEL::FF);
    shooter_1.SetTarget(-SHOOTER_WHEEL::SHOOTING_RPM, rev::ControlType::kVelocity);

}

[[depricated]] void ShooterWheel::bangbang() //origional code with commented code removed
{   
    return;
    // if(abs(shooter_encoder.GetVelocity() > SHOOTER_WHEEL::SHOOTING_RPM - 1200))
        // shooter_1.SetOpenLoopRampRate(0);
    // else
        // shooter_1.SetOpenLoopRampRate(6);
// 
    // if(abs(shooter_encoder.GetVelocity() > SHOOTER_WHEEL::SHOOTING_RPM - 2000))
        // shooter_1.SetOpenLoopRampRate(0);
    // else
        // shooter_1.SetOpenLoopRampRate(6);
    // if((abs(shooter_encoder.GetVelocity()) < SHOOTER_WHEEL::SHOOTING_RPM - 250))
        // shooter_1.Set(-1);
    // else
        // shooter_1.Set(0);
}