#include "Auton.hpp"
#include "Robot.hpp"
#include "Timer.hpp"
#include "RobotState.hpp"


void Auton::ThreeBall()
{
    using namespace AUTO::THREE_BALL;
    using namespace RobotState;

    ngr::Timer timer;
    ngr::Timer spinup_timer;

    spinup_timer.Reset();
    spinup_timer.Start();

    timer.Reset();
    timer.Start();
    while(timer.Get() < DRIVE_TIME && IsAutonomous() && IsEnabled())
    {
        Drivetrain::drive(frc::ChassisSpeeds { 0_mps, -.25_mps * WHEELS::speed_mult, 0_rad_per_s });
        aim(TURRET::POSITION::BACK);
        std::this_thread::sleep_for(10ms);
    }
    Drivetrain::gotoZero();

// This needs to be updated with the new photon code anyhow once we merge photon
//    camera.setLEDMode(PhotonCamera::LED_Mode::Force_On);
    timer.Reset();
    timer.Start();
    while(IsAutonomous() && IsEnabled())
    {
        std::this_thread::sleep_for(10ms);
        if(aim(TURRET::POSITION::BACK) && spinup_timer.Get() > SPINUP_TIME && timer.Get() < SHOOT_WAIT_TIME)
            Hopper::shoot();
    }
}