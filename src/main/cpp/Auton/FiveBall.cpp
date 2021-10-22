#include "Auton.hpp"
#include "Robot.hpp"
#include "Timer.hpp"
#include "RobotState.hpp"


void Auton::FiveBall()
{
    using namespace AUTO::FIVE_BALL;
    using namespace RobotState;

    ngr::Timer timer;

    // drive back / intake
    Intake::deploy(true);
    Intake::drive(INTAKE::DIRECTION::IN);

    // move to balls
    Drivetrain::drive({ 0_mps * WHEELS::speed_mult,
                        -0.35_mps * WHEELS::speed_mult,
                        .0_rad_per_s });
    std::this_thread::sleep_for(MOVE_TO_BALLS);

    // pickup balls
    Drivetrain::drive({ -.2_mps * WHEELS::speed_mult,
                        -.10_mps * WHEELS::speed_mult,
                        .1_rad_per_s });
    std::this_thread::sleep_for(PICKUP_TIME);

    // move to goal
    Drivetrain::drive({ .4_mps * WHEELS::speed_mult,
                        .1_mps * WHEELS::speed_mult,
                        0_rad_per_s });
    std::this_thread::sleep_for(PICKUP_TIME);


    // don't drive forever
    Drivetrain::gotoZero();

    // shoot
    timer.Reset();
    timer.Start();
 //   camera.setLEDMode(PhotonCamera::LED_Mode::Force_On);
    while(IsAutonomous() && IsEnabled())
    {
        std::this_thread::sleep_for(10ms);
        if(aim(TURRET::POSITION::BACK) && timer.Get() > SHOOT_WAIT_TIME)
            Hopper::shoot();
    }
}