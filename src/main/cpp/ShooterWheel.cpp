#include "ShooterWheel.hpp"
#include <rev/CANSparkMax.h>
#include <rev/CANEncoder.h>


inline static rev::CANSparkMax shooter_1 { SHOOTER_WHEEL::PORT_1, rev::CANSparkMaxLowLevel::MotorType::kBrushless };
inline static rev::CANEncoder  shooter_encoder = shooter_1.GetEncoder();
inline static bool             maxSpeedForShooting;

/******************************************************************/
/*                      Non Static Functions                      */
/******************************************************************/

void ShooterWheel::init()
{
    // shooter_1.RestoreFactoryDefaults();
    shooter_1.SetIdleMode(SHOOTER_WHEEL::IDLE_MODE);
}

void ShooterWheel::bangbang() //original code with commented code removed
{
    // return;
    // if(abs(shooter_encoder.GetVelocity() > SHOOTER_WHEEL::SHOOTING_RPM - 1200))
    //     shooter_1.SetOpenLoopRampRate(0);
    // else
    //     shooter_1.SetOpenLoopRampRate(6);

    // if(abs(shooter_encoder.GetVelocity() > SHOOTER_WHEEL::SHOOTING_RPM - 2000))
    if(maxSpeedForShooting)
    {
    }
    else
    {
        shooter_1.SetOpenLoopRampRate(0);
        // else
        //     shooter_1.SetOpenLoopRampRate(3);

        if((abs(shooter_encoder.GetVelocity()) < 2000))
            shooter_1.Set(-.5);
        else if((abs(shooter_encoder.GetVelocity()) < SHOOTER_WHEEL::SHOOTING_RPM))
            shooter_1.Set(-1),
                printf("1\n");
        else
            shooter_1.Set(0), printf("0\n");
    }
}

double ShooterWheel::get_speed()
{
    return shooter_encoder.GetVelocity();
}

void ShooterWheel::stop()
{
    shooter_1.Set(0);
}

double ShooterWheel::get_temp()
{
    return shooter_1.GetMotorTemperature();
}

// True overrides bangbang, false returns to normal function
void ShooterWheel::enableMaxSpeedForShooting(bool boolean)
{
    maxSpeedForShooting = boolean;
}
