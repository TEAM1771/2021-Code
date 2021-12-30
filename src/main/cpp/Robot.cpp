#include "Robot.hpp"
#include "Timer.hpp"
#include "LimeLight.hpp"
#include "RobotState.hpp"
#include "ShooterWheel.hpp"
#include "Turret.hpp"
#include "Climber.hpp"
#include "Drivetrain.hpp"
#include "Hood.hpp"
#include "Hopper.hpp"
#include "Intake.hpp"
#include "Buttons.hpp"
#include "Wheel.hpp"
#include <frc/Timer.h>
#include <frc/smartdashboard/smartdashboard.h>
#include <memory>

/* This section of code is used with PhotonLib Example 3 but idk where to put it in the actual code
Source: https://docs.photonvision.org/en/latest/docs/examples/simaimandrange.html
#include "PLExampleCode/3_TargetAimRange.hpp"
void Robot::SimulationPeriodic() {
    dtSim.update();
}
*/
LimeLight camera {};
double    adjust_shooter       = .5;
bool      overheating_flash_red = true;
//Average<20> averageCameraY;


Robot::Robot()
{
    // setup RobotStates
    RobotState::isEnabled                = [this]() { return IsEnabled(); };
    RobotState::isDisabled               = [this]() { return IsDisabled(); };
    RobotState::isAutonomous             = [this]() { return IsAutonomous(); };
    RobotState::isAutonomousEnabled      = [this]() { return IsAutonomousEnabled(); };
    RobotState::isOperatorControl        = [this]() { return IsOperatorControl(); };
    RobotState::isOperatorControlEnabled = [this]() { return IsOperatorControlEnabled(); };
    RobotState::isTest                   = [this]() { return IsTest(); };


    Climber::init();
    Drivetrain::init();
    Hood::init();
    Hopper::init();
    Intake::init();
    Turret::init();
    ShooterWheel::init();

    shooterTempUpdate();
}

namespace AUTO //In the future, might move all of this over to a seperate auton folder or something
{
    using namespace std::literals::chrono_literals;
    namespace THREE_BALL
    {

        constexpr auto SPINUP_TIME      = 4s;
        constexpr auto DRIVE_TIME       = 1.8s;
        constexpr auto SHOOT_WAIT_TIME  = 2s;
        constexpr auto SHOOT_TOTAL_TIME = SHOOT_WAIT_TIME + 3s;

    } // namespace THREE_BALL

    namespace FIVE_BALL
    {
        constexpr auto MOVE_TO_BALLS     = 2.3s;
        constexpr auto MOVE_TO_GOAL_TIME = 1.75s;
        constexpr auto SHOOT_WAIT_TIME   = 1s;
        constexpr auto SHOOT_TOTAL_TIME  = SHOOT_WAIT_TIME + 1.5s;
    } // namespace FIVE_BALL
    namespace SIX_BALL
    {
        constexpr auto SPIN_UP_TIME      = 4.5s;
        constexpr auto PICKUP_DRIVE_TIME = 2.8s;
        constexpr auto SHOOT_TIME_1      = 1s;
        constexpr auto TIME_BACKWARD     = 2.3s;
        constexpr auto SHOOT_WAIT_TIME   = 1.5s;
    } // namespace SIX_BALL


    namespace EIGHT_BALL
    {
        constexpr auto WAIT_BETWEEN_TURNS  = 0.15s;
        constexpr auto MOVE_STRAIGHT_BACK  = 1s;
        constexpr auto MOVE_BACK_AND_TURN  = 1.1s;
        constexpr auto PICKUP_SECOND_THREE = 1.4375s;
        constexpr auto ALIGN_WITH_GOAL     = 0.8s;
        constexpr auto SECOND_MOVE_TO_GOAL = 1.1s;
        constexpr auto STOP_AND_AIM_TIME   = 0.4s;
        constexpr auto SECOND_SHOOT_TIME   = 1.5s;
        inline auto    keepAiming          = true;
        //constexpr auto secondVolleyShooterY = .5;
    } // namespace EIGHT_BALL
} // namespace AUTO

void Robot::threeBall()
{
    using namespace AUTO::THREE_BALL;

    ngr::Timer timer;

    std::this_thread::sleep_for(SPINUP_TIME);
    Drivetrain::autonDrive(-0.35_mps * WHEELS::speed_mult,
                            0_mps * WHEELS::speed_mult,
                            -180_deg);
    std::this_thread::sleep_for(DRIVE_TIME);
    Drivetrain::stop();

    timer.Reset();
    timer.Start();
    camera.setLEDMode(LimeLight::LED_Mode::Force_On);
    while(IsAutonomous() && IsEnabled() && timer.Get() < SHOOT_TOTAL_TIME)
    {
        std::this_thread::sleep_for(10ms);
        if(aim(Turret::POSITION::BACK) && timer.Get() > SHOOT_WAIT_TIME)
            Hopper::shoot();
    }
    Hopper::stop();
    Turret::goToPosition(Turret::POSITION::ZERO);
    Drivetrain::stop();
}

void Robot::fiveBall()
{
    using namespace AUTO::FIVE_BALL;
    ngr::Timer timer;

    // drive back / intake
    Intake::deploy(true);
    Intake::drive(Intake::DIRECTION::IN);

    // move to balls
    Drivetrain::autonDrive(0_mps * WHEELS::speed_mult,
                            -0.35_mps * WHEELS::speed_mult,
                            -180_deg);
    std::this_thread::sleep_for(MOVE_TO_BALLS);
    Drivetrain::stop();
    Intake::drive(Intake::DIRECTION::OFF);

    // Everytime we see sleep_for() we can try to reduce this
    std::this_thread::sleep_for(AUTO::EIGHT_BALL::WAIT_BETWEEN_TURNS);

    // move to goal
    Drivetrain::autonDrive(-.175_mps * WHEELS::speed_mult,
                            .3_mps * WHEELS::speed_mult,
                            0_deg);
    std::this_thread::sleep_for(MOVE_TO_GOAL_TIME);
    Drivetrain::stop();
    Intake::drive(Intake::DIRECTION::IN);

    // shoot
    timer.Reset();
    timer.Start();
    camera.setLEDMode(LimeLight::LED_Mode::Force_On);
    while(IsAutonomous() && IsEnabled() && timer.Get() < SHOOT_TOTAL_TIME)
    {
        std::this_thread::sleep_for(10ms);
        if(aim(Turret::POSITION::FRONT) && timer.Get() > SHOOT_WAIT_TIME)
            Hopper::shoot();
    }
    Hopper::stop();
}


void Robot::sixBall()
{
    using namespace AUTO::SIX_BALL;
    ngr::Timer timer;


    // drive back / intake
    Intake::deploy(true);
    Intake::drive(Intake::DIRECTION::IN);

    timer.Reset();
    timer.Start();
    while(timer.Get() < SPIN_UP_TIME)
        aim(Turret::POSITION::BACK);

    // std::this_thread::sleep_for(SPIN_UP_TIME);

    // std::thread aim_and_shoot_ { [this] {
    //     ngr::Timer
    //             timer;
    timer.Reset();
    timer.Start();
    camera.setLEDMode(LimeLight::LED_Mode::Force_On);
    while(timer.Get() < SHOOT_TIME_1 && IsAutonomous() && IsEnabled())
    {
        std::this_thread::sleep_for(10ms);
        if(aim(Turret::POSITION::BACK))
            Hopper::shoot();
    }
    Hopper::stop();
    // } };

    timer.Reset();
    timer.Start();
    while(timer.Get() < PICKUP_DRIVE_TIME && IsAutonomous() && IsEnabled())
    {
        Drivetrain::autonDrive(0_mps,
                                -0.45_mps * WHEELS::speed_mult,
                                0_rad);
        std::this_thread::sleep_for(20ms); // don't spam the CAN network
    }

    // turn and move back toward goal
    Drivetrain::autonDrive(-0.15_mps * WHEELS::speed_mult, ////////////////////////////////////////////////////
                            0.45_mps * WHEELS::speed_mult,  ////////////////////////////////////////////////////////
                            0_rad);
    std::this_thread::sleep_for(TIME_BACKWARD);
    Drivetrain::stop();


    timer.Reset();
    timer.Start();
    std::thread aim_and_shoot { [this, timer] {
        camera.setLEDMode(LimeLight::LED_Mode::Force_On);
        while(IsAutonomous() && IsEnabled())
        {
            std::this_thread::sleep_for(10ms);
            if(aim(Turret::POSITION::BACK) && timer.Get() > SHOOT_WAIT_TIME)
                Hopper::shoot();
        }
    } };


    // wait for threads to exit
    aim_and_shoot.join();
}


void Robot::eightBall()
{
    using namespace AUTO::EIGHT_BALL;
    ngr::Timer timer;

    fiveBall(); //Pickup 2, shoot first 5

    //Drive straight backwards
    Drivetrain::autonDrive(0_mps,
                            -0.35_mps * WHEELS::speed_mult,
                            0_deg);
    std::this_thread::sleep_for(MOVE_STRAIGHT_BACK);

    //Continues driving back but begins to turn and move to the left
    Drivetrain::autonDrive(-0.1_mps * WHEELS::speed_mult,
                            -0.35_mps,
                            30_deg);
    std::this_thread::sleep_for(MOVE_BACK_AND_TURN);
    Drivetrain::stop();

    std::this_thread::sleep_for(WAIT_BETWEEN_TURNS);

    //Now begins going forward & faster to the left, picking up three
    Drivetrain::autonDrive(-0.14168_mps * WHEELS::speed_mult,
                            0.224_mps,
                            30_deg);
    std::this_thread::sleep_for(PICKUP_SECOND_THREE);
    Drivetrain::stop();
    Intake::drive(Intake::DIRECTION::OFF);

    std::this_thread::sleep_for(WAIT_BETWEEN_TURNS);

    //Move to the right to avoid pole
    Drivetrain::autonDrive(0.3_mps * WHEELS::speed_mult,
                            0_mps,
                            0_deg);
    std::this_thread::sleep_for(ALIGN_WITH_GOAL);
    Drivetrain::stop();

    std::this_thread::sleep_for(WAIT_BETWEEN_TURNS);

    //Now drive straight forward to the goal
    Drivetrain::autonDrive(0_mps * WHEELS::speed_mult,
                            0.25_mps,
                            0_deg);

    //Start aiming and prepare to shoot
    timer.Reset();
    timer.Start();
    std::thread aim_and_shoot { [this, timer] {
        camera.setLEDMode(LimeLight::LED_Mode::Force_On);
        while(IsAutonomous() && IsEnabled() && timer.Get() < (SECOND_MOVE_TO_GOAL + STOP_AND_AIM_TIME + SECOND_SHOOT_TIME))
        {
            std::this_thread::sleep_for(10ms);
            Intake::drive(Intake::DIRECTION::IN);
            if(aim(Turret::POSITION::FRONT) && timer.Get() >= SECOND_MOVE_TO_GOAL + STOP_AND_AIM_TIME)
                Hopper::shoot();
        }
    } };

    //Stop when it arrives at shoot location in front of goal
    std::this_thread::sleep_for(SECOND_MOVE_TO_GOAL);
    Drivetrain::stop();


    /*
    If previous solution for second volley doesn't work, try inputing manual interpolation value from test
    if(auto [is_tracking, readyToShoot] = Turret::visionTrack(direction); is_tracking)
          Hood::manualPositionControl(secondVolleyShooterY); && readyToShoot;
    else
        Hood::goToPosition(Hood::POSITION::TRAVERSE);
   
    std::this_thread::sleep_for(STOP_AND_AIM_TIME);
    timer.Reset();
    timer.Start();
    camera.setLEDMode(LimeLight::LED_Mode::Force_On);
    while(IsAutonomous() && IsEnabled() && timer.Get() < SECOND_SHOOT_TIME)
    {
        std::this_thread::sleep_for(10ms);
        
        if(timer.Get() > SHOOT_WAIT_TIME)
            Hopper::shoot();
    }
    Hopper::stop();
    */


    //Stop aiming/shooting (handled in aim_and_shoot)
    std::this_thread::sleep_for(STOP_AND_AIM_TIME + SECOND_SHOOT_TIME);
    aim_and_shoot.join();

    //Just to make sure the intake doesn't hit limelight again...
    Turret::goToPosition(Turret::POSITION::ZERO);
    Hopper::stop();
}

void Robot::AutonomousInit()
{
    Drivetrain::resetGyro();
    using namespace std::literals::chrono_literals;

    // Drivetrain::autonDrive(0_mps,
    //                         -0.25_mps * WHEELS::speed_mult,
    //                         0_deg);
    // std::this_thread::sleep_for(2s);
    // Drivetrain::stop();
    // return;

    // Start BangBang and indexer
    std::thread run_shooter_wheel_and_index_balls { [this] {
        using namespace std::literals::chrono_literals;
        while(IsAutonomous() && IsEnabled())
        {
            ShooterWheel::bangbang();
            Hopper::index(false);             // don't warn when called while shooting
            std::this_thread::sleep_for(5ms); // don't spam the CAN network
        }
    } };
    Intake::deploy(true);

    Drivetrain::goToZero();
    std::this_thread::sleep_for(0.25s);
    // threeBall();
    //sixBall();
    //fiveBall();
    eightBall();

    run_shooter_wheel_and_index_balls.join();
}

void Robot::AutonomousPeriodic()
{
    shooterTempUpdate();
}

void Robot::TeleopInit()
{
    //camera.debug();
    Hopper::stop(); // eliminates need to shoot at start of teleop
}
void Robot::TeleopPeriodic()
{
    shooterTempUpdate();

    if(BUTTON::oStick.GetThrottle() > 0)
    {
        ShooterWheel::bangbang();
    }
    else
    {
        ShooterWheel::stop();
    }
    // printf("speed: %f\n", ShooterWheel::getSpeed());
    buttonManager();

    //("\n CamY: %f\tAngle: %f", averageCameraY(Hood::getCameraY()), Hood::getAngle());
    //printf("\n CamY: %f\tAngle: %f", Hood::getCameraY(), Hood::getAngle());
    //printf("\n Shooter Temp: %f", ShooterWheel::getTemp());
}
void Robot::TestInit()
{
}

void Robot::TestPeriodic()
{
    shooterTempUpdate();
    if(BUTTON::RUMBLE)
    {
        BUTTON::ps5.SetRumble(BUTTON::ps5.kLeftRumble, .7);
        BUTTON::ps5.SetRumble(BUTTON::ps5.kRightRumble, .7);
    }
    else
    {
        BUTTON::ps5.SetRumble(BUTTON::ps5.kLeftRumble, 0);
        BUTTON::ps5.SetRumble(BUTTON::ps5.kRightRumble, 0);
    }


    /*
    ShooterWheel::bangbang();
    printf("CamY: %f\tAngle: %f", Hood::getCameraY(), Hood::getAngle());
    printf("\n Shooter Temp: %f", ShooterWheel::getTemp());
    //Hood::manualPositionControl(BUTTON::oStick.GetThrottle());

    if(BUTTON::SHOOTER::ADJUST_SHOOTER_UP.getRawButtonPressed())
    {
        adjust_shooter += 0.01;
        if(adjust_shooter > 1)
            adjust_shooter = 1;
    }
    else if(BUTTON::SHOOTER::ADJUST_SHOOTER_DOWN.getRawButtonPressed())
    {
        adjust_shooter -= 0.01;
        if(adjust_shooter < 0)
            adjust_shooter = 0;
    }

    if(adjust_shooter > 1)
        adjust_shooter = 1;
    if(adjust_shooter < 0)
        adjust_shooter = 0;
    Hood::manualPositionControl(adjust_shooter);

    Intake::deploy(true);
    auto targetLocked = Turret::visionTrack(TURRET::BACK);

    if(BUTTON::SHOOTER::SHOOT.getRawButtonReleased())
        Hopper::stop();
    if(targetLocked.readyToShoot && BUTTON::SHOOTER::SHOOT)
        Hopper::shoot();
    else if(! BUTTON::SHOOTER::SHOOT)
        Hopper::index();
*/

    // double x = BUTTON::ps5.GetX() * WHEELS::speed_mult;

    // if(fabs(x) < .1)
    //     x = 0;
    // double y = -BUTTON::ps5.GetY() * WHEELS::speed_mult;

    // if(fabs(y) < .1)
    //     y = 0;

    // double rotate = BUTTON::ps5.GetZ() * 2;
    // if(fabs(rotate) < .1)
    //     rotate = 0;


    // if(BUTTON::ps5.GetRawButton(4))
    //     Drivetrain::faceDirection( units::meters_per_second_t(0), units::meters_per_second_t(.2), 0_deg );
    // else if(BUTTON::ps5.GetRawButton(2))
    //     Drivetrain::faceDirection( units::meters_per_second_t(0), units::meters_per_second_t(-.2), 0_deg );
    // else Drivetrain::stop();
    // else
    // if(BUTTON::DRIVETRAIN::ROTATE_FRONT)
    //     Drivetrain::faceDirection(units::meters_per_second_t { x }, units::meters_per_second_t { y }, 0_deg);
    // if(BUTTON::DRIVETRAIN::ROTATE_BACK)
    //     Drivetrain::faceDirection(units::meters_per_second_t { x }, units::meters_per_second_t { y }, 180_deg);
    // if(BUTTON::DRIVETRAIN::ROTATE_TO_CLOSEST)
    //     Drivetrain::faceClosest(units::meters_per_second_t { x }, units::meters_per_second_t { y });
}

void Robot::DisabledInit()
{
}

void Robot::DisabledPeriodic()
{
    shooterTempUpdate();
    //printf("\n CamY: %f\tAngle: %f", averageCameraY(Hood::getCameraY()), Hood::getAngle());
    //printf("\n CamY: %f\tAngle: %f", Hood::getCameraY(), Hood::getAngle());
    //printf("\n Shooter Temp: %f", ShooterWheel::getTemp());
}

void Robot::buttonManager()
{
    bool target_locked = false;
    bool deploy_intake = false;
    if(BUTTON::SHOOTER::AIM_FRONT)
    {
        deploy_intake = true;
        target_locked = aim(Turret::POSITION::FRONT);
    }
    else if(BUTTON::SHOOTER::AIM_BACK)
    {
        deploy_intake = true;
        target_locked = aim(Turret::POSITION::BACK);
    }
    else if(BUTTON::SHOOTER::BATTERSHOT)
    {
        deploy_intake = true;

        // turret_in_pos is true when it's safe to deploy hood
        bool const turret_in_pos = Turret::goToPosition(Turret::POSITION::FRONT,
                                                        ngr::fabs(Turret::POSITION::FRONT - Turret::POSITION::SAFE_TO_DEPLOY_HOOD_FRONT));
        if(turret_in_pos)
            target_locked = Hood::goToPosition(Hood::POSITION::BATTER);
        else
            Hood::goToPosition(Hood::POSITION::TRAVERSE);
    }
    else if(BUTTON::SHOOTER::AIM_SIDE)
    {
        deploy_intake = true;
        target_locked = Hood::goToPosition(Hood::POSITION::MIDPOINT);
    }
    else
    {
        deploy_intake = false;
        if(Hood::goToPosition(Hood::POSITION::BOTTOM, ngr::fabs(Hood::POSITION::SAFE_TO_TURN)))
            Turret::goToPosition(Turret::POSITION::ZERO);
    }

    Intake::deploy(BUTTON::INTAKE::DEPLOY || deploy_intake);

    if(BUTTON::SHOOTER::SHOOT.getRawButtonReleased())
        Hopper::stop();
    if(target_locked && BUTTON::SHOOTER::SHOOT)
        Hopper::shoot();
    else if(! BUTTON::SHOOTER::SHOOT)
        Hopper::index();

    if(BUTTON::INTAKE::INTAKE)
        Intake::drive(Intake::DIRECTION::IN);
    else if(BUTTON::INTAKE::RETRACT)
        Intake::drive(Intake::DIRECTION::OUT);
    else
        Intake::drive(Intake::DIRECTION::OFF);

    Climber::buttonManager();


    double x = BUTTON::ps5.GetX() * WHEELS::speed_mult;
    if(fabs(x) < .04)
        x = 0;
    double y = -BUTTON::ps5.GetY() * WHEELS::speed_mult;
    if(fabs(y) < .04)
        y = 0;
    // double RjoyX = BUTTON::ps5.GetZ();
    // double RjoyY = BUTTON::ps5.GetTwist();

    // double joystickAngle = std::atan2(RjoyY, RjoyX);


    double rotate = BUTTON::ps5.GetZ() * 2;
    if(fabs(rotate) < .1)
        rotate = 0;

    // if(BUTTON::DRIVETRAIN::ZERO)
    //     Drivetrain::goToZero();
    // else if(BUTTON::DRIVETRAIN::REVERSE)
    //     Drivetrain::goTo180();
    // else

    if(BUTTON::DRIVETRAIN::ROTATE_FRONT)
        Drivetrain::faceDirection(units::meters_per_second_t { x }, units::meters_per_second_t { y }, 0_deg);
    // if(BUTTON::DRIVETRAIN::ROTATE_BACK)
    //     Drivetrain::faceDirection(units::meters_per_second_t { x }, units::meters_per_second_t { y }, 180_deg);
    // else if(BUTTON::DRIVETRAIN::ROTATE_TO_CLOSEST)
    //     Drivetrain::faceClosest(units::meters_per_second_t { x }, units::meters_per_second_t { y });
    else if(BUTTON::DRIVETRAIN::ROTATE_CLIMB)
        Drivetrain::faceDirection(units::meters_per_second_t { x }, units::meters_per_second_t { y }, 67.5_deg);
    // }
    else
        Drivetrain::drive(frc::ChassisSpeeds { units::meters_per_second_t { x },
                                               units::meters_per_second_t { y },
                                               units::radians_per_second_t { rotate } });
    // printf("rotate: %f\n", rotate);
}

bool Robot::aim(Turret::POSITION direction)
{
    if(auto [is_tracking, readyToShoot] = Turret::visionTrack(direction); is_tracking)
        return Hood::visionTrack() && readyToShoot;
    Hood::goToPosition(Hood::POSITION::TRAVERSE);
    return false;
}
bool Robot::shooterTempUpdate()
{
    frc::SmartDashboard::PutNumber("Shooter Temp", ShooterWheel::getTemp());
    printf("\n Shooter Temp: %f", ShooterWheel::getTemp());
    if(ShooterWheel::getTemp() > 70)
    {
        // oscillating between green & red to grab attention
        if(overheating_flash_red)
        {
            frc::SmartDashboard::PutBoolean("Shooter (not) Overheating", false);
            overheating_flash_red = false;
        }
        else
        {
            frc::SmartDashboard::PutBoolean("Shooter (not) Overheating", true);
            overheating_flash_red = true;
        }
        // BUTTON::ps5.SetRumble(BUTTON::ps5.kLeftRumble, .7);
        // BUTTON::ps5.SetRumble(BUTTON::ps5.kRightRumble, .7);
    }
    else
    {
        frc::SmartDashboard::PutBoolean("Shooter (not) Overheating", true);
        // BUTTON::ps5.SetRumble(BUTTON::ps5.kLeftRumble, 0);
        // BUTTON::ps5.SetRumble(BUTTON::ps5.kRightRumble, 0);
    }
    return ShooterWheel::getTemp() > 70;
}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
