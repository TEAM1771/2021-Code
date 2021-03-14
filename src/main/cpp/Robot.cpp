#include "Robot.hpp"
#include "Timer.hpp"

void Robot::ThreeBall()
{
    using namespace AUTO::THREE_BALL;

    ngr::Timer timer;

    // Start BangBang and indexer
    std::thread run_shooter_wheel { [this] {
        while(IsAutonomous() && IsEnabled())
        {
            shooter_wheel.bangbang();
            std::this_thread::sleep_for(5ms); // don't spam the CAN network
        }
    } };

    while(drivetrain.driveDistanceForward(drive_distance) && IsAutonomous() && IsEnabled())
    {
        aim(TURRET::POSITION::BACK);
        std::this_thread::sleep_for(10ms);
    }


    limelight.setLEDMode(LimeLight::LED_Mode::Force_On);
    while(IsAutonomous() && IsEnabled())
    {
        std::this_thread::sleep_for(10ms);
        if(aim(TURRET::POSITION::BACK) && timer.Get() > minimum_shoot_time)
            hopper.shoot();
    }

    run_shooter_wheel.join();
}

void Robot::FiveBall()
{
    using namespace AUTO::FIVE_BALL;

    // Start BangBang and indexer
    std::thread run_shooter_wheel_and_index_balls { [this] {
        using namespace std::literals::chrono_literals;
        while(IsAutonomous() && IsEnabled())
        {
            shooter_wheel.bangbang();
            hopper.index(false);              // don't warn when called while shooting
            std::this_thread::sleep_for(5ms); // don't spam the CAN network
        }
    } };

    // drive back / intake
    intake.deploy(true);
    intake.drive(INTAKE::DIRECTION::IN);
    drivetrain.reset();
    while(! drivetrain.driveDistanceForward(PICKUP_DISTANCE))
        std::this_thread::sleep_for(20ms); // don't spam the CAN network

    // turn
    drivetrain.drive(.3, 0);
    std::this_thread::sleep_for(TURN_TIME);
    // drive forward and target back, shoot when ready
    drivetrain.drive(.5, .5);
    std::thread aim_and_shoot { [this] {
        limelight.setLEDMode(LimeLight::LED_Mode::Force_On);
        while(IsAutonomous() && IsEnabled())
        {
            std::this_thread::sleep_for(10ms);
            if(aim(TURRET::POSITION::BACK))
                hopper.shoot();
        }
    } };
    std::this_thread::sleep_for(TIME_BACKWARD);
    drivetrain.drive(0, 0);

    // wait for threads to exit
    run_shooter_wheel_and_index_balls.join();
    aim_and_shoot.join();
}

void Robot::AutonomousInit()
{
    FiveBall();
}

void Robot::AutonomousPeriodic()
{
}

void Robot::TeleopInit()
{
    hopper.stop(); // eliminates need to shoot at start of teleop
}

void Robot::TeleopPeriodic()
{
    shooter_wheel.bangbang();
    ButtonManager();
    drivetrain.drive(BUTTON::lStick.GetY(), BUTTON::rStick.GetY());
    drivetrain.shift();
}

void Robot::TestPeriodic()
{
    intake.deploy(true);
    turret.visionTrack(TURRET::POSITION::BACK);
}

void Robot::DisabledInit()
{
}

void Robot::DisabledPeriodic()
{
}

void Robot::ButtonManager()
{
    bool targetLocked = false;
    bool deployIntake = false;
    if(BUTTON::SHOOTER::AIM_FRONT)
    {
        deployIntake = true;
        targetLocked = aim(TURRET::POSITION::FRONT);
    }
    else if(BUTTON::SHOOTER::AIM_BACK)
    {
        deployIntake = true;
        targetLocked = aim(TURRET::POSITION::BACK);
    }
    else if(BUTTON::SHOOTER::BATTERSHOT)
    {
        deployIntake = true;

        // turret_in_pos is true when it's safe to deploy hood
        bool const turret_in_pos = turret.goToPosition(TURRET::POSITION::FRONT,
                                                       fabs(TURRET::POSITION::FRONT - TURRET::POSITION::SAFE_TO_DEPLOY_HOOD_FRONT));
        if(turret_in_pos)
            targetLocked = hood.goToPosition(HOOD::POSITION::BATTER);
        else
            hood.goToPosition(HOOD::POSITION::TRAVERSE);
    }
    else if(BUTTON::SHOOTER::AIM_SIDE)
    {
        deployIntake = true;
        targetLocked = hood.goToPosition(HOOD::POSITION::MIDPOINT);
    }
    else
    {
        deployIntake = false;
        if(hood.goToPosition(HOOD::POSITION::BOTTOM, fabs(HOOD::POSITION::SAFE_TO_TURN)))
            turret.goToPosition(TURRET::POSITION::ZERO);
    }

    intake.deploy(BUTTON::INTAKE::DEPLOY || deployIntake);

    if(BUTTON::SHOOTER::SHOOT.getRawButtonReleased())
        hopper.stop();
    if(targetLocked && BUTTON::SHOOTER::SHOOT)
        hopper.shoot();
    else if(! BUTTON::SHOOTER::SHOOT)
        hopper.index();

    if(BUTTON::INTAKE::INTAKE)
        intake.drive(INTAKE::DIRECTION::IN);
    else if(BUTTON::INTAKE::RETRACT)
        intake.drive(INTAKE::DIRECTION::OUT);
    else
        intake.drive(INTAKE::DIRECTION::OFF);

    climber.ButtonManager();
}

bool Robot::aim(TURRET::POSITION direction)
{
    if(auto [is_tracking, readyToShoot] = turret.visionTrack(direction); is_tracking)
        return hood.visionTrack() && readyToShoot;
    hood.goToPosition(HOOD::POSITION::TRAVERSE);
    return false;
}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
