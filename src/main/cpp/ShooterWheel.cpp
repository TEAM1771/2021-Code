#include "ShooterWheel.hpp"

//private (static) variables
inline static rev::CANSparkMax shooter_1 { SHOOTER_WHEEL::PORT_1, rev::CANSparkMaxLowLevel::MotorType::kBrushless };
inline static rev::CANEncoder  shooter_encoder = shooter_1.GetEncoder();

//public function definitions
void ShooterWheel::init()
{
    // shooter_1.RestoreFactoryDefaults();
    shooter_1.SetIdleMode(SHOOTER_WHEEL::IDLE_MODE);
}

void ShooterWheel::bangbang() //origional code with commented code removed
{
    // return;
    // if(abs(shooter_encoder.GetVelocity() > SHOOTER_WHEEL::SHOOTING_RPM - 1200))
    //     shooter_1.SetOpenLoopRampRate(0);
    // else
    //     shooter_1.SetOpenLoopRampRate(6);

    // if(abs(shooter_encoder.GetVelocity() > SHOOTER_WHEEL::SHOOTING_RPM - 2000))
    shooter_1.SetOpenLoopRampRate(0);
    // else
    //     shooter_1.SetOpenLoopRampRate(3);

    if((abs(shooter_encoder.GetVelocity()) < 2000))
        shooter_1.Set(-.75);
    else if((abs(shooter_encoder.GetVelocity()) < SHOOTER_WHEEL::SHOOTING_RPM))
        shooter_1.Set(-1),
            printf("1\n");
    else
        shooter_1.Set(0), printf("0\n");
}

double ShooterWheel::get_speed()
{
    return shooter_encoder.GetVelocity();
}