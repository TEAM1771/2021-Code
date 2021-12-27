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
#include <frc/Timer.h>
#include <frc/smartdashboard/smartdashboard.h>
#include <memory>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>

/* This section of code is used with PhotonLib Example 3 but idk where to put it in the actual code
Source: https://docs.photonvision.org/en/latest/docs/examples/simaimandrange.html
#include "PLExampleCode/3_TargetAimRange.hpp"
void Robot::SimulationPeriodic() {
    dtSim.update();
}
*/
LimeLight camera {};
double    adjustShooter       = .5;
bool      overheatingFlashRed = true;
//Average<20> averageCameraY;


Robot::Robot()
{
    // setup RobotStates
    RobotState::IsEnabled                = [this]() { return IsEnabled(); };
    RobotState::IsDisabled               = [this]() { return IsDisabled(); };
    RobotState::IsAutonomous             = [this]() { return IsAutonomous(); };
    RobotState::IsAutonomousEnabled      = [this]() { return IsAutonomousEnabled(); };
    RobotState::IsOperatorControl        = [this]() { return IsOperatorControl(); };
    RobotState::IsOperatorControlEnabled = [this]() { return IsOperatorControlEnabled(); };
    RobotState::IsTest                   = [this]() { return IsTest(); };


    Climber::init();
    Drivetrain::init();
    Hood::init();
    Hopper::init();
    Intake::init();
    Turret::init();
    ShooterWheel::init();

    ShooterTempUpdate();
}

void Robot::ThreeBall()
{
    using namespace AUTO::THREE_BALL;

    ngr::Timer timer;

    std::this_thread::sleep_for(SPINUP_TIME);
    Drivetrain::auton_drive(-0.35_mps * WHEELS::speed_mult,
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
        if(aim(TURRET::POSITION::BACK) && timer.Get() > SHOOT_WAIT_TIME)
            Hopper::shoot();
    }
    Hopper::stop();
    Turret::goToPosition(TURRET::POSITION::ZERO);
    Drivetrain::stop();
}

void Robot::FiveBall()
{
    using namespace AUTO::FIVE_BALL;
    ngr::Timer timer;

    // drive back / intake
    Intake::deploy(true);
    Intake::drive(INTAKE::DIRECTION::IN);

    // move to balls
    Drivetrain::auton_drive(0_mps * WHEELS::speed_mult,
                            -0.35_mps * WHEELS::speed_mult,
                            -180_deg);
    std::this_thread::sleep_for(MOVE_TO_BALLS);
    Drivetrain::stop();
    Intake::drive(INTAKE::DIRECTION::OFF);

    // Everytime we see sleep_for() we can try to reduce this
    std::this_thread::sleep_for(AUTO::EIGHT_BALL::WAIT_BETWEEN_TURNS);

    // move to goal
    Drivetrain::auton_drive(-.175_mps * WHEELS::speed_mult,
                            .3_mps * WHEELS::speed_mult,
                            0_deg);
    std::this_thread::sleep_for(MOVE_TO_GOAL_TIME);
    Drivetrain::stop();
    Intake::drive(INTAKE::DIRECTION::IN);

    // shoot
    timer.Reset();
    timer.Start();
    camera.setLEDMode(LimeLight::LED_Mode::Force_On);
    while(IsAutonomous() && IsEnabled() && timer.Get() < SHOOT_TOTAL_TIME)
    {
        std::this_thread::sleep_for(10ms);
        if(aim(TURRET::POSITION::FRONT) && timer.Get() > SHOOT_WAIT_TIME)
            Hopper::shoot();
    }
    Hopper::stop();
}


void Robot::SixBall()
{
    using namespace AUTO::SIX_BALL;
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
    camera.setLEDMode(LimeLight::LED_Mode::Force_On);
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
        Drivetrain::auton_drive(0_mps,
                                -0.45_mps * WHEELS::speed_mult,
                                0_rad);
        std::this_thread::sleep_for(20ms); // don't spam the CAN network
    }

    // turn and move back toward goal
    Drivetrain::auton_drive(-0.15_mps * WHEELS::speed_mult, ////////////////////////////////////////////////////
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
            if(aim(TURRET::POSITION::BACK) && timer.Get() > SHOOT_WAIT_TIME)
                Hopper::shoot();
        }
    } };


    // wait for threads to exit
    aim_and_shoot.join();
}


void Robot::EightBall()
{
    using namespace AUTO::EIGHT_BALL;
    ngr::Timer timer;

    FiveBall(); //Pickup 2, shoot first 5

    //Drive straight backwards
    Drivetrain::auton_drive(0_mps,
                            -0.35_mps * WHEELS::speed_mult,
                            0_deg);
    std::this_thread::sleep_for(MOVE_STRAIGHT_BACK);

    //Continues driving back but begins to turn and move to the left
    Drivetrain::auton_drive(-0.1_mps * WHEELS::speed_mult,
                            -0.35_mps,
                            30_deg);
    std::this_thread::sleep_for(MOVE_BACK_AND_TURN);
    Drivetrain::stop();

    std::this_thread::sleep_for(WAIT_BETWEEN_TURNS);

    //Now begins going forward & faster to the left, picking up three
    Drivetrain::auton_drive(-0.14168_mps * WHEELS::speed_mult,
                            0.224_mps,
                            30_deg);
    std::this_thread::sleep_for(PICKUP_SECOND_THREE);
    Drivetrain::stop();
    Intake::drive(INTAKE::DIRECTION::OFF);

    std::this_thread::sleep_for(WAIT_BETWEEN_TURNS);

    //Move to the right to avoid pole
    Drivetrain::auton_drive(0.3_mps * WHEELS::speed_mult,
                            0_mps,
                            0_deg);
    std::this_thread::sleep_for(ALIGN_WITH_GOAL);
    Drivetrain::stop();

    std::this_thread::sleep_for(WAIT_BETWEEN_TURNS);

    //Now drive straight forward to the goal
    Drivetrain::auton_drive(0_mps * WHEELS::speed_mult,
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
            Intake::drive(INTAKE::DIRECTION::IN);
            if(aim(TURRET::POSITION::FRONT) && timer.Get() >= SECOND_MOVE_TO_GOAL + STOP_AND_AIM_TIME)
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
        Hood::goToPosition(HOOD::POSITION::TRAVERSE);
   
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
    Turret::goToPosition(TURRET::POSITION::ZERO);
    Hopper::stop();
}

void Robot::TestTrajectory()
{
    using namespace DRIVETRAIN::TRAJECTORY;
    auto config = frc::TrajectoryConfig(3_mps, 3_mps);
    config.SetKinematics<4>(const_cast<frc::SwerveDriveKinematics<4>&>(Drivetrain::get_kinematics()));
    // auto startPos = frc::Pose2d(0_m, 0_m, frc::Rotation2d(0));
    frc::Pose2d const startPos;
    frc::Pose2d const endPos{3_m, 3_m, 180_deg};
    std::vector<frc::Translation2d> interiorPos{
        frc::Translation2d{1.5_m, 1_m},
        frc::Translation2d{2_m, 2.5_m}
    };

    auto traj = frc::TrajectoryGenerator::GenerateTrajectory(startPos, interiorPos, endPos, config);

    Drivetrain::trajectory_auton_drive(traj);
}

void Robot::AutonomousInit()
{
    Drivetrain::reset_gyro();
    using namespace std::literals::chrono_literals;

    // Drivetrain::auton_drive(0_mps,
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

    Drivetrain::gotoZero();
    std::this_thread::sleep_for(0.25s);
    // ThreeBall();
    //SixBall();
    //FiveBall();
    EightBall();

    run_shooter_wheel_and_index_balls.join();
}

void Robot::AutonomousPeriodic()
{
    ShooterTempUpdate();
}

void Robot::TeleopInit()
{
    //camera.debug();
    Hopper::stop(); // eliminates need to shoot at start of teleop
}
void Robot::TeleopPeriodic()
{
    ShooterTempUpdate();

    Drivetrain::update_odometry();

    if(BUTTON::oStick.GetThrottle() > 0)
    {
        ShooterWheel::bangbang();
    }
    else
    {
        ShooterWheel::stop();
    }
    // printf("speed: %f\n", ShooterWheel::get_speed());
    ButtonManager();

    //("\n CamY: %f\tAngle: %f", averageCameraY(Hood::get_camera_Y()), Hood::get_angle());
    //printf("\n CamY: %f\tAngle: %f", Hood::get_camera_Y(), Hood::get_angle());
    //printf("\n Shooter Temp: %f", ShooterWheel::get_temp());
}
void Robot::TestInit()
{
}

void Robot::TestPeriodic()
{
    ShooterTempUpdate();
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
    printf("CamY: %f\tAngle: %f", Hood::get_camera_Y(), Hood::get_angle());
    printf("\n Shooter Temp: %f", ShooterWheel::get_temp());
    //Hood::manualPositionControl(BUTTON::oStick.GetThrottle());

    if(BUTTON::SHOOTER::ADJUST_SHOOTER_UP.getRawButtonPressed())
    {
        adjustShooter += 0.01;
        if(adjustShooter > 1)
            adjustShooter = 1;
    }
    else if(BUTTON::SHOOTER::ADJUST_SHOOTER_DOWN.getRawButtonPressed())
    {
        adjustShooter -= 0.01;
        if(adjustShooter < 0)
            adjustShooter = 0;
    }

    if(adjustShooter > 1)
        adjustShooter = 1;
    if(adjustShooter < 0)
        adjustShooter = 0;
    Hood::manualPositionControl(adjustShooter);

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
    //     Drivetrain::face_direction( units::meters_per_second_t(0), units::meters_per_second_t(.2), 0_deg );
    // else if(BUTTON::ps5.GetRawButton(2))
    //     Drivetrain::face_direction( units::meters_per_second_t(0), units::meters_per_second_t(-.2), 0_deg );
    // else Drivetrain::stop();
    // else
    // if(BUTTON::DRIVETRAIN::ROTATE_FRONT)
    //     Drivetrain::face_direction(units::meters_per_second_t { x }, units::meters_per_second_t { y }, 0_deg);
    // if(BUTTON::DRIVETRAIN::ROTATE_BACK)
    //     Drivetrain::face_direction(units::meters_per_second_t { x }, units::meters_per_second_t { y }, 180_deg);
    // if(BUTTON::DRIVETRAIN::ROTATE_TO_CLOSEST)
    //     Drivetrain::face_closest(units::meters_per_second_t { x }, units::meters_per_second_t { y });
}

void Robot::DisabledInit()
{
}

void Robot::DisabledPeriodic()
{
    ShooterTempUpdate();
    //printf("\n CamY: %f\tAngle: %f", averageCameraY(Hood::get_camera_Y()), Hood::get_angle());
    //printf("\n CamY: %f\tAngle: %f", Hood::get_camera_Y(), Hood::get_angle());
    //printf("\n Shooter Temp: %f", ShooterWheel::get_temp());
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
        bool const turret_in_pos = Turret::goToPosition(TURRET::POSITION::FRONT,
                                                        ngr::fabs(TURRET::POSITION::FRONT - TURRET::POSITION::SAFE_TO_DEPLOY_HOOD_FRONT));
        if(turret_in_pos)
            targetLocked = Hood::goToPosition(HOOD::POSITION::BATTER);
        else
            Hood::goToPosition(HOOD::POSITION::TRAVERSE);
    }
    else if(BUTTON::SHOOTER::AIM_SIDE)
    {
        deployIntake = true;
        targetLocked = Hood::goToPosition(HOOD::POSITION::MIDPOINT);
    }
    else
    {
        deployIntake = false;
        if(Hood::goToPosition(HOOD::POSITION::BOTTOM, ngr::fabs(HOOD::POSITION::SAFE_TO_TURN)))
            Turret::goToPosition(TURRET::POSITION::ZERO);
    }

    Intake::deploy(BUTTON::INTAKE::DEPLOY || deployIntake);

    if(BUTTON::SHOOTER::SHOOT.getRawButtonReleased())
        Hopper::stop();
    if(targetLocked && BUTTON::SHOOTER::SHOOT)
        Hopper::shoot();
    else if(! BUTTON::SHOOTER::SHOOT)
        Hopper::index();

    if(BUTTON::INTAKE::INTAKE)
        Intake::drive(INTAKE::DIRECTION::IN);
    else if(BUTTON::INTAKE::RETRACT)
        Intake::drive(INTAKE::DIRECTION::OUT);
    else
        Intake::drive(INTAKE::DIRECTION::OFF);

    Climber::ButtonManager();


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
    //     Drivetrain::gotoZero();
    // else if(BUTTON::DRIVETRAIN::REVERSE)
    //     Drivetrain::goto180();
    // else

    if(BUTTON::DRIVETRAIN::ROTATE_FRONT)
        Drivetrain::face_direction(units::meters_per_second_t { x }, units::meters_per_second_t { y }, 0_deg);
    // if(BUTTON::DRIVETRAIN::ROTATE_BACK)
    //     Drivetrain::face_direction(units::meters_per_second_t { x }, units::meters_per_second_t { y }, 180_deg);
    // else if(BUTTON::DRIVETRAIN::ROTATE_TO_CLOSEST)
    //     Drivetrain::face_closest(units::meters_per_second_t { x }, units::meters_per_second_t { y });
    else if(BUTTON::DRIVETRAIN::ROTATE_CLIMB)
        Drivetrain::face_direction(units::meters_per_second_t { x }, units::meters_per_second_t { y }, 67.5_deg);
    // }
    else
        Drivetrain::drive(frc::ChassisSpeeds { units::meters_per_second_t { x },
                                               units::meters_per_second_t { y },
                                               units::radians_per_second_t { rotate } });
    // printf("rotate: %f\n", rotate);
}

bool Robot::aim(TURRET::POSITION direction)
{
    if(auto [is_tracking, readyToShoot] = Turret::visionTrack(direction); is_tracking)
        return Hood::visionTrack() && readyToShoot;
    Hood::goToPosition(HOOD::POSITION::TRAVERSE);
    return false;
}
bool Robot::ShooterTempUpdate()
{
    frc::SmartDashboard::PutNumber("Shooter Temp", ShooterWheel::get_temp());
    printf("\n Shooter Temp: %f", ShooterWheel::get_temp());
    if(ShooterWheel::get_temp() > 70)
    {
        // oscillating between green & red to grab attention
        if(overheatingFlashRed)
        {
            frc::SmartDashboard::PutBoolean("Shooter (not) Overheating", false);
            overheatingFlashRed = false;
        }
        else
        {
            frc::SmartDashboard::PutBoolean("Shooter (not) Overheating", true);
            overheatingFlashRed = true;
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
    return ShooterWheel::get_temp() > 70;
}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
