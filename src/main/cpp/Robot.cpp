#include "Robot.hpp"
#include "Timer.hpp"
//#include "PhotonVision.hpp"
#include "LimeLight.hpp"
#include "RobotState.hpp"
/* This section of code is used with PhotonLib Example 3 but idk where to put it in the actual code
Source: https://docs.photonvision.org/en/latest/docs/examples/simaimandrange.html
#include "PLExampleCode/3_TargetAimRange.hpp"
void Robot::SimulationPeriodic() {
    dtSim.update();
}
*/
LimeLight camera {};

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
}

void Robot::ThreeBall()
{
    using namespace AUTO::THREE_BALL;

    ngr::Timer timer;
    ngr::Timer spinup_timer;

    spinup_timer.Reset();
    spinup_timer.Start();

    timer.Reset();
    timer.Start();
    while(timer.Get() < DRIVE_TIME && IsAutonomous() && IsEnabled())
    {
        Drivetrain::drive({ 0_mps, -.25_mps * WHEELS::speed_mult, 0_rad_per_s });

        aim(TURRET::POSITION::BACK);
        std::this_thread::sleep_for(10ms);
    }
    Drivetrain::stop();

    camera.setLEDMode(LimeLight::LED_Mode::Force_On);
    timer.Reset();
    timer.Start();
    while(IsAutonomous() && IsEnabled())
    {
        std::this_thread::sleep_for(10ms);
        if(aim(TURRET::POSITION::BACK) && spinup_timer.Get() > SPINUP_TIME && timer.Get() < SHOOT_WAIT_TIME)
            Hopper::shoot();
    }
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
                            .0_rad);
    std::this_thread::sleep_for(MOVE_TO_BALLS);

    // pickup balls
    Drivetrain::auton_drive(-.2_mps * WHEELS::speed_mult,
                            -.10_mps * WHEELS::speed_mult,
                            .1_rad);
    std::this_thread::sleep_for(PICKUP_TIME);

    // move to goal
    Drivetrain::auton_drive(.4_mps * WHEELS::speed_mult,
                            .1_mps * WHEELS::speed_mult,
                            0_rad);
    std::this_thread::sleep_for(PICKUP_TIME);


    // don't drive forever
    Drivetrain::stop();

    // shoot
    timer.Reset();
    timer.Start();
    camera.setLEDMode(LimeLight::LED_Mode::Force_On);
    while(IsAutonomous() && IsEnabled())
    {
        std::this_thread::sleep_for(10ms);
        if(aim(TURRET::POSITION::BACK) && timer.Get() > SHOOT_WAIT_TIME)
            Hopper::shoot();
    }
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


    Intake::deploy(true);
    Intake::drive(INTAKE::DIRECTION::IN);


    /////////////////////////
    // Shoot before moving //
    /////////////////////////
    while(ShooterWheel::get_speed() < SHOOTER_WHEEL::SHOOTING_RPM - 500 && IsAutonomous() && IsEnabled())
    {
        std::cout << "waiting for shooter wheel\n";
        std::this_thread::sleep_for(10ms);
    }
    std::cout << "shooter wheel ready\n";

    camera.setLEDMode(LimeLight::LED_Mode::Force_On);
    timer.Reset();
    timer.Start();
    while(timer.Get() < SHOOT_TIME_1 && IsAutonomous() && IsEnabled())
    {
        std::this_thread::sleep_for(10ms);
        if(aim(TURRET::POSITION::BACK))
            Hopper::shoot();
    }

    // Go to traverse
    while(IsAutonomous() && IsEnabled() &&
          ! (Hood::goToPosition(HOOD::POSITION::BOTTOM, ngr::fabs(HOOD::POSITION::SAFE_TO_TURN)) && Turret::goToPosition(TURRET::POSITION::ZERO)))
        std::this_thread::sleep_for(10ms);

    ///////////////////////
    // Pickup Trench Run //
    ///////////////////////
    Drivetrain::auton_drive(0_mps,
                            0.3_mps * WHEELS::speed_mult,
                            0_rad);
    std::this_thread::sleep_for(TRENCH_RUN_PICKUP_TIME);


    Drivetrain::auton_drive(0_mps,
                            0.5_mps * WHEELS::speed_mult,
                            0_rad);
    std::this_thread::sleep_for(TRENCH_RUN_RETURN_TIME);
    Drivetrain::stop();

    camera.setLEDMode(LimeLight::LED_Mode::Force_On);
    timer.Reset();
    timer.Start();
    while(timer.Get() < SHOOT_TIME_2 && IsAutonomous() && IsEnabled())
    {
        std::this_thread::sleep_for(10ms);
        if(aim(TURRET::POSITION::BACK))
            Hopper::shoot();
    }
}

void Robot::TenBall()
{
    using namespace AUTO::TEN_BALL;
    ngr::Timer timer;

    FiveBall();

    Drivetrain::auton_drive(0_mps,
                            -0.5_mps * WHEELS::speed_mult,
                            0.5_rad);
    std::this_thread::sleep_for(RETURN_PICKUP_TIME);
    Drivetrain::auton_drive(-.3_mps * WHEELS::speed_mult,
                            0_mps,
                            0_rad);
    std::this_thread::sleep_for(PICKUP_MOVE_TIME);
    Drivetrain::auton_drive(.3_mps * WHEELS::speed_mult,
                            0_mps,
                            0_rad);
    std::this_thread::sleep_for(PICKUP_RETURN_TIME);

    Drivetrain::auton_drive(0_mps,
                            0.5_mps * WHEELS::speed_mult,
                            -0.5_rad);
    std::this_thread::sleep_for(GOAL_RETURN_TIME);
    while(IsAutonomous() && IsEnabled())
    {
        std::this_thread::sleep_for(10ms);
        if(aim(TURRET::POSITION::BACK))
            Hopper::shoot();
    }
}

void Robot::ThirteenBall()
{
    using namespace AUTO::THIRTEEN_BALL;
    ngr::Timer timer;

    EightBall();

    Drivetrain::auton_drive(0_mps,
                            -0.5_mps * WHEELS::speed_mult,
                            0.5_rad);
    std::this_thread::sleep_for(RETURN_PICKUP_TIME);
    Drivetrain::auton_drive(-.3_mps * WHEELS::speed_mult,
                            0_mps,
                            0_rad);
    std::this_thread::sleep_for(PICKUP_MOVE_TIME);
    Drivetrain::auton_drive(.3_mps * WHEELS::speed_mult,
                            0_mps,
                            0_rad);
    std::this_thread::sleep_for(PICKUP_RETURN_TIME);

    Drivetrain::auton_drive(0_mps,
                            0.5_mps * WHEELS::speed_mult,
                            -0.5_rad);
    std::this_thread::sleep_for(GOAL_RETURN_TIME);
    while(IsAutonomous() && IsEnabled())
    {
        std::this_thread::sleep_for(10ms);
        if(aim(TURRET::POSITION::BACK))
            Hopper::shoot();
    }
}

void Robot::AutonomousInit()
{
    Drivetrain::reset_gyro();
    using namespace std::literals::chrono_literals;

    Drivetrain::gotoZero();
    std::this_thread::sleep_for(0.1s);
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

    ThreeBall();
    //SixBall();
    //FiveBall();

    run_shooter_wheel_and_index_balls.join();
}

void Robot::AutonomousPeriodic()
{
}

void Robot::TeleopInit()
{
    //camera.debug();
    Hopper::stop(); // eliminates need to shoot at start of teleop
}
void Robot::TeleopPeriodic()
{
    if(BUTTON::oStick.GetThrottle() < 0)
    {
        ShooterWheel::bangbang();
    }
    else
    {
        ShooterWheel::stop();
    }
    // printf("speed: %f\n", ShooterWheel::get_speed());
    ButtonManager();
}
void Robot::TestInit()
{
}

void Robot::TestPeriodic()
{
    ShooterWheel::bangbang();
    printf("CamY: %f\tAngle: %f", Hood::get_camera_Y(), Hood::get_angle());
    Hood::manualPositionControl(BUTTON::oStick.GetThrottle());
    Intake::deploy(true);
    auto targetLocked = Turret::visionTrack(TURRET::BACK);

    if(BUTTON::SHOOTER::SHOOT.getRawButtonReleased())
        Hopper::stop();
    if(targetLocked.readyToShoot && BUTTON::SHOOTER::SHOOT)
        Hopper::shoot();
    else if(! BUTTON::SHOOTER::SHOOT)
        Hopper::index();



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
    else if(BUTTON::DRIVETRAIN::ROTATE_BACK)
        Drivetrain::face_direction(units::meters_per_second_t { x }, units::meters_per_second_t { y }, 180_deg);
    // else if(BUTTON::DRIVETRAIN::ROTATE_TO_CLOSEST)
    //     Drivetrain::face_closest(units::meters_per_second_t { x }, units::meters_per_second_t { y });
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

#ifndef RUNNING_FRC_TESTS
int main()
{
            return frc::StartRobot<Robot>();
}
#endif
