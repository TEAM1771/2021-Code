#include "Auton.hpp"
#include "Robot.hpp"
#include "Timer.hpp"
#include "RobotState.hpp"


void Auton::SixBall()
{
    using namespace AUTO::SIX_BALL;
    using namespace RobotState;

    ngr::Timer timer;


    // drive back / intake
    Intake::deploy(true);
    Intake::drive(INTAKE::DIRECTION::IN);

    timer.Reset();
    timer.Start();
    while(timer.Get() < SPIN_UP_TIME)
        aim(TURRET::POSITION::BACK);

    // std::this_thread::sleep_for(SPIN_UP_TIME);

    // std::thread aim_and_shoot_ { [this] {
    //     ngr::Timer
    //             timer;
    timer.Reset();
    timer.Start();
//    camera.setLEDMode(PhotonCamera::LED_Mode::Force_On);
    while(timer.Get() < SHOOT_TIME_1 && IsAutonomous() && IsEnabled())
    {
        std::this_thread::sleep_for(10ms);
        if(aim(TURRET::POSITION::BACK))
            Hopper::shoot();
    }
    Hopper::stop();
    // } };

    timer.Reset();
    timer.Start();
    while(timer.Get() < PICKUP_DRIVE_TIME && IsAutonomous() && IsEnabled())
    {
        Drivetrain::drive({ 0_mps,
                            -0.45_mps * WHEELS::speed_mult,
                            0_rad_per_s });
        std::this_thread::sleep_for(20ms); // don't spam the CAN network
    }

    // turn and move back toward goal
    Drivetrain::drive({ -0.15_mps * WHEELS::speed_mult, ////////////////////////////////////////////////////
                        0.45_mps * WHEELS::speed_mult,  ////////////////////////////////////////////////////////
                        0_rad_per_s });
    std::this_thread::sleep_for(TIME_BACKWARD);
    Drivetrain::gotoZero();


    timer.Reset();
    timer.Start();
    std::thread aim_and_shoot { [timer] {
//        camera.setLEDMode(PhotonCamera::LED_Mode::Force_On);
        while(IsAutonomous() && IsEnabled())
        {
            std::this_thread::sleep_for(10ms);
            if(aim(TURRET::POSITION::BACK) && timer.Get() > SHOOT_WAIT_TIME)
                Hopper::shoot();
        }
    } };


    // wait for threads to exit
    aim_and_shoot.join();
}