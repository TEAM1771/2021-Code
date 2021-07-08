#include "Robot.hpp"
#include "Timer.hpp"

void Robot::ThreeBall()
{
    using namespace AUTO::THREE_BALL;

    ngr::Timer timer;

    timer.Reset();
    timer.Start();
    while(timer.Get() < DRIVE_FORWARD_TIME && IsAutonomous() && IsEnabled())
    {
        drivetrain.drive(frc::ChassisSpeeds { 0_mps, -.25_mps * WHEELS::speed_mult, 0_rad_per_s });
        aim(TURRET::POSITION::BACK);
        std::this_thread::sleep_for(10ms);
    }
    drivetrain.gotoZero();


    while(shooter_wheel.get_speed() < SHOOTER_WHEEL::SHOOTING_RPM - 500 && IsAutonomous() && IsEnabled())
    {
        std::cout << "waiting for shooter wheel\n";
        std::this_thread::sleep_for(10ms);
    }
    std::cout << "shooter wheel ready\n";

    limelight.setLEDMode(LimeLight::LED_Mode::Force_On);
    timer.Reset();
    timer.Start();
    while(IsAutonomous() && IsEnabled())
    {
        std::this_thread::sleep_for(10ms);
        if(aim(TURRET::POSITION::BACK) && timer.Get() > MINIMUM_SHOOT_TIME)
            hopper.shoot();
    }
}

void Robot::FiveBall()
{
    using namespace AUTO::FIVE_BALL;
    ngr::Timer timer;


    // drive back / intake
    intake.deploy(true);
    intake.drive(INTAKE::DIRECTION::IN);
    timer.Reset();
    timer.Start();
    while(timer.Get() < PICKUP_DRIVE_TIME && IsAutonomous() && IsEnabled())
    {
        drivetrain.drive({ 0_mps,
                           -0.35_mps * WHEELS::speed_mult,
                           0_rad_per_s });
        std::this_thread::sleep_for(20ms); // don't spam the CAN network
    }

    // turn and move back toward goal
    drivetrain.drive({ -0.1_mps * WHEELS::speed_mult,
                       0.35_mps * WHEELS::speed_mult,
                       0_rad_per_s });

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
    drivetrain.gotoZero();

    // wait for threads to exit
    aim_and_shoot.join();
}


void Robot::SixBall()
{
    using namespace AUTO::SIX_BALL;
    ngr::Timer timer;


    // drive back / intake
    intake.deploy(true);
    intake.drive(INTAKE::DIRECTION::IN);


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
    limelight.setLEDMode(LimeLight::LED_Mode::Force_On);
    while(timer.Get() < SHOOT_TIME_1 && IsAutonomous() && IsEnabled())
    {
        std::this_thread::sleep_for(10ms);
        if(aim(TURRET::POSITION::BACK))
            hopper.shoot();
    }
    hopper.stop();
    // } };

    timer.Reset();
    timer.Start();
    while(timer.Get() < PICKUP_DRIVE_TIME && IsAutonomous() && IsEnabled())
    {
        drivetrain.drive({ 0_mps,
                           -0.45_mps * WHEELS::speed_mult,
                           0_rad_per_s });
        std::this_thread::sleep_for(20ms); // don't spam the CAN network
    }

    // turn and move back toward goal
    drivetrain.drive({ -0.15_mps * WHEELS::speed_mult, ////////////////////////////////////////////////////
                       0.45_mps * WHEELS::speed_mult,  ////////////////////////////////////////////////////////
                       0_rad_per_s });
    std::this_thread::sleep_for(TIME_BACKWARD);
    drivetrain.gotoZero();


    timer.Reset();
    timer.Start();
    std::thread aim_and_shoot { [this, timer] {
        limelight.setLEDMode(LimeLight::LED_Mode::Force_On);
        while(IsAutonomous() && IsEnabled())
        {
            std::this_thread::sleep_for(10ms);
            if(aim(TURRET::POSITION::BACK) && timer.Get() < SHOOT_WAIT_TIME)
                hopper.shoot();
        }
    } };


    // wait for threads to exit
    aim_and_shoot.join();
}


void Robot::EightBall()
{
    using namespace AUTO::EIGHT_BALL;
    ngr::Timer timer;


    intake.deploy(true);
    intake.drive(INTAKE::DIRECTION::IN);


    /////////////////////////
    // Shoot before moving //
    /////////////////////////
    while(shooter_wheel.get_speed() < SHOOTER_WHEEL::SHOOTING_RPM - 500 && IsAutonomous() && IsEnabled())
    {
        std::cout << "waiting for shooter wheel\n";
        std::this_thread::sleep_for(10ms);
    }
    std::cout << "shooter wheel ready\n";

    limelight.setLEDMode(LimeLight::LED_Mode::Force_On);
    timer.Reset();
    timer.Start();
    while(timer.Get() < SHOOT_TIME_1 && IsAutonomous() && IsEnabled())
    {
        std::this_thread::sleep_for(10ms);
        if(aim(TURRET::POSITION::BACK))
            hopper.shoot();
    }

    // Go to traverse
    while(IsAutonomous() && IsEnabled() &&
          ! (hood.goToPosition(HOOD::POSITION::BOTTOM, fabs(HOOD::POSITION::SAFE_TO_TURN)) && turret.goToPosition(TURRET::POSITION::ZERO)))
        std::this_thread::sleep_for(10ms);

    ///////////////////////
    // Pickup Trench Run //
    ///////////////////////
    drivetrain.drive({ 0_mps,
                       0.3_mps * WHEELS::speed_mult,
                       0_rad_per_s });
    std::this_thread::sleep_for(TRENCH_RUN_PICKUP_TIME);


    drivetrain.drive({ 0_mps,
                       0.5_mps * WHEELS::speed_mult,
                       0_rad_per_s });
    std::this_thread::sleep_for(TRENCH_RUN_RETURN_TIME);
    drivetrain.gotoZero();

    limelight.setLEDMode(LimeLight::LED_Mode::Force_On);
    timer.Reset();
    timer.Start();
    while(timer.Get() < SHOOT_TIME_2 && IsAutonomous() && IsEnabled())
    {
        std::this_thread::sleep_for(10ms);
        if(aim(TURRET::POSITION::BACK))
            hopper.shoot();
    }
}

void Robot::TenBall()
{
    using namespace AUTO::TEN_BALL;
    ngr::Timer timer;

    FiveBall();

    drivetrain.drive({ 0_mps,
                       -0.5_mps * WHEELS::speed_mult,
                       0.5_rad_per_s });
    std::this_thread::sleep_for(RETURN_PICKUP_TIME);
    drivetrain.drive({ -.3_mps * WHEELS::speed_mult,
                       0_mps,
                       0_rad_per_s });
    std::this_thread::sleep_for(PICKUP_MOVE_TIME);
    drivetrain.drive({ .3_mps * WHEELS::speed_mult,
                       0_mps,
                       0_rad_per_s });
    std::this_thread::sleep_for(PICKUP_RETURN_TIME);

    drivetrain.drive({ 0_mps,
                       0.5_mps * WHEELS::speed_mult,
                       -0.5_rad_per_s });
    std::this_thread::sleep_for(GOAL_RETURN_TIME);
    while(IsAutonomous() && IsEnabled())
    {
        std::this_thread::sleep_for(10ms);
        if(aim(TURRET::POSITION::BACK))
            hopper.shoot();
    }
}

void Robot::ThirteenBall()
{
    using namespace AUTO::THIRTEEN_BALL;
    ngr::Timer timer;

    EightBall();

    drivetrain.drive({ 0_mps,
                       -0.5_mps * WHEELS::speed_mult,
                       0.5_rad_per_s });
    std::this_thread::sleep_for(RETURN_PICKUP_TIME);
    drivetrain.drive({ -.3_mps * WHEELS::speed_mult,
                       0_mps,
                       0_rad_per_s });
    std::this_thread::sleep_for(PICKUP_MOVE_TIME);
    drivetrain.drive({ .3_mps * WHEELS::speed_mult,
                       0_mps,
                       0_rad_per_s });
    std::this_thread::sleep_for(PICKUP_RETURN_TIME);

    drivetrain.drive({ 0_mps,
                       0.5_mps * WHEELS::speed_mult,
                       -0.5_rad_per_s });
    std::this_thread::sleep_for(GOAL_RETURN_TIME);
    while(IsAutonomous() && IsEnabled())
    {
        std::this_thread::sleep_for(10ms);
        if(aim(TURRET::POSITION::BACK))
            hopper.shoot();
    }
}

void Robot::AutonomousInit()
{
    drivetrain.reset_gyro();
    using namespace std::literals::chrono_literals;

    drivetrain.gotoZero();
    std::this_thread::sleep_for(0.1s);
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
    intake.deploy(true);

    SixBall();


    run_shooter_wheel_and_index_balls.join();
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
    // printf("speed: %f\n", shooter_wheel.get_speed());
    ButtonManager();
}
void Robot::TestInit()
{
}

void Robot::TestPeriodic()
{
    // printf("angle: %f\n", drivetrain.get_angle());

    // climber.printStatus();
    // drivetrain.PrintWheelAngle(2);
    printf("CamY: %f\tAngle: ", hood.get_camera_Y(), hood.get_angle());
    hood.manualPositionControl(BUTTON::oStick.GetThrottle());

    auto targetLocked = turret.visionTrack(TURRET::BACK);

    if(BUTTON::SHOOTER::SHOOT.getRawButtonReleased())
        hopper.stop();
    if(targetLocked.readyToShoot && BUTTON::SHOOTER::SHOOT)
        hopper.shoot();
    else if(! BUTTON::SHOOTER::SHOOT)
        hopper.index();

    // intake.deploy(true);
    // turret.visionTrack(TURRET::POSITION::BACK);
    //drivetrain.print();
    // shooter_wheel.bangbang();
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

    double x = BUTTON::ps5.GetX() * WHEELS::speed_mult;
    if(fabs(x) < .1)
        x = 0;
    double y = -BUTTON::ps5.GetY() * WHEELS::speed_mult;
    if(fabs(y) < .1)
        y = 0;
    // double RjoyX = BUTTON::ps5.GetZ();
    // double RjoyY = BUTTON::ps5.GetTwist();

    // double joystickAngle = std::atan2(RjoyY, RjoyX);


    double rotate = BUTTON::ps5.GetZ() * 2;
    if(fabs(rotate) < .1)
        rotate = 0;

    // if(BUTTON::DRIVETRAIN::ZERO)
    //     drivetrain.gotoZero();
    // else if(BUTTON::DRIVETRAIN::REVERSE)
    //     drivetrain.goto180();
    // else
    drivetrain.drive(frc::ChassisSpeeds { units::meters_per_second_t { x },
                                          units::meters_per_second_t { y },
                                          units::radians_per_second_t { rotate } });
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
