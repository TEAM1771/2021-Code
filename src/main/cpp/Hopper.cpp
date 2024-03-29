#include "Hopper.hpp"
#include "ShooterWheel.hpp"

#include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>

#include <cmath>

using can_adr = int;
/******************************************************************/
/*                             Constants                          */
/******************************************************************/
namespace INDEXER
{
    const can_adr PORT  = 10;
    const double  SPEED = 0.7;

    const auto IDLE_MODE = rev::CANSparkMax::IdleMode::kBrake;
} // namespace INDEXER

namespace TRANSPORT
{
    const can_adr PORT = 3;

    const auto IDLE_MODE = rev::CANSparkMax::IdleMode::kBrake;

    const double SPEED       = 0.4;
    const double SHOOT_SPEED = 1.0; //previous value was 1.0

    const double DISTANCE  = 79.0 / 3;
    const double TOLERANCE = 1;

    const double P = 0.3;
    const double I = 0;
    const double D = 0.0001;
} // namespace TRANSPORT

using can_adr = int;
/******************************************************************/
/*                          Non-constant Vars                     */
/******************************************************************/

const can_adr LIMIT_SWITCH = 0;

inline static rev::CANSparkMax      indexer { INDEXER::PORT, rev::CANSparkMaxLowLevel::MotorType::kBrushless };
inline static rev::CANSparkMax      transport { TRANSPORT::PORT, rev::CANSparkMaxLowLevel::MotorType::kBrushless };
inline static rev::CANPIDController pid_controller = transport.GetPIDController();
inline static rev::CANEncoder       encoder       = transport.GetEncoder();
inline static frc::DigitalInput     limit_switch { LIMIT_SWITCH };
inline static int                   number_of_balls  = 3;
inline static double                target_distance = TRANSPORT::DISTANCE;
inline static bool                  is_transporting = false;
inline static std::atomic<bool>     invalid_stop_flag { false };


/******************************************************************/
/*                      Non Static Functions                      */
/******************************************************************/

void Hopper::init()
{
    // indexer.Set(INDEXER::SPEED);


    indexer.SetIdleMode(INDEXER::IDLE_MODE);
    indexer.SetSmartCurrentLimit(20);

    transport.SetIdleMode(TRANSPORT::IDLE_MODE);
    transport.SetSmartCurrentLimit(40);

    pid_controller.SetP(TRANSPORT::P);
    pid_controller.SetI(TRANSPORT::I);
    pid_controller.SetD(TRANSPORT::D);

    pid_controller.SetFeedbackDevice(encoder);
    pid_controller.SetOutputRange(-TRANSPORT::SPEED, TRANSPORT::SPEED);

    encoder.SetPosition(0);
}

bool Hopper::index(bool warn_if_shooting)
{
    if(invalid_stop_flag)
    {
        if(warn_if_shooting)
            std::cerr << "Stop not called after shooting: Indexer Aborting\n";
        return false;
    }

    if(! limit_switch.Get() && number_of_balls < 3 && ! is_transporting)
    {
        pid_controller.SetReference(target_distance, rev::ControlType::kPosition);
        number_of_balls++;
        is_transporting = true;
    }

    if(is_transporting && encoder.GetPosition() > (target_distance - TRANSPORT::TOLERANCE))
    {
        target_distance += TRANSPORT::DISTANCE;
        is_transporting = false;
    }

    if(limit_switch.Get() && number_of_balls < 4)
        indexer.Set(INDEXER::SPEED);
    else
        indexer.Set(0);
    return true;
}

void Hopper::shoot()
{
    invalid_stop_flag = true;
    indexer.Set(INDEXER::SPEED - 0.3);
    ShooterWheel::setShooting(true);
    transport.Set(TRANSPORT::SHOOT_SPEED);
}

void Hopper::stop()
{
    ShooterWheel::setShooting(false);
    if(invalid_stop_flag)
    {
        invalid_stop_flag = false;
        is_transporting  = false;
        number_of_balls   = 0;
        target_distance  = TRANSPORT::DISTANCE;
        encoder.SetPosition(0);

        transport.Set(0);
    }
}
