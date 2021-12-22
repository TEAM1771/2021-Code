#include "drivetrain.hpp"
//#include <frc/kinematics/ChassisSpeeds.h> Replaced by next line
// #include <frc/kinematics/SwerveModuleState.h> Replaced by next line
#include "frc/kinematics/SwerveDriveOdometry.h"

#include "Twist.hpp"
#include "Wheel.hpp"
#include "RobotState.hpp"
#include <AHRS.h>
#include <array>
#include <frc/geometry/Translation2d.h>
#include <thread>
#include <iostream>
#include <memory>
#include "frc/controller/HolonomicDriveController.h"

// #include <cmath>

inline static std::array<std::unique_ptr<Wheel>, 4> wheels {
    std::make_unique<Wheel>(WHEELS::WHEEL_1),
    std::make_unique<Wheel>(WHEELS::WHEEL_2),
    std::make_unique<Wheel>(WHEELS::WHEEL_3),
    std::make_unique<Wheel>(WHEELS::WHEEL_4)
};

inline static frc::SwerveDriveKinematics<4> const m_kinematics {
    *wheels[0],
    *wheels[1],
    *wheels[2],
    *wheels[3]
};

inline static frc::SwerveDriveOdometry const m_odometry { m_kinematics, Drivetrain::get_heading() };

inline static std::unique_ptr<AHRS> navx { std::make_unique<AHRS>(frc::SPI::Port::kMXP) };

// In the future some values should be moved to Constants.hpp and changed as needed
inline static frc::HolonomicDriveController controller {
    frc2::PIDController { 1, 0, 0 }, // 1 meter in X direction for every meter of error
    frc2::PIDController { 1, 0, 0 }, // 1 meter in Y direction for every meter of error
    frc::ProfiledPIDController<units::radian> {
        1,
        0,
        0,
        frc::TrapezoidProfile<units::radian>::Constraints {
            6.28_rad_per_s,
            3.14_rad_per_s / 1_s } }
}; // Max Velocity of 1 rotation/sec, max acceleration of pi / second^2

/*
    void initDriveController(double xErrorCorrection, double yErrorCorrection,
    double zErrorCorrection, double maxRotationalVelocity, double maxRotationalAcceleration);
*/

/******************************************************************/
/*                      Non Static Functions                      */
/******************************************************************/

void Drivetrain::init()
{
    reset_gyro();
}

void Drivetrain::reset_gyro()
{
    navx->ZeroYaw();
}

double Drivetrain::get_angle()
{
    return -navx->GetAngle() + 90; // This should be replaced with - 90 and we should fix all the auton_drive methods
}

frc::Rotation2d Drivetrain::get_heading()
{
    return { units::degree_t { -get_angle() } };
}

void Drivetrain::drive(frc::ChassisSpeeds const& field_speeds)
{
    auto const speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        field_speeds.vx,
        field_speeds.vy,
        field_speeds.omega,
        frc::Rotation2d { units::degree_t { get_angle() } });
    auto const module_states = m_kinematics.ToSwerveModuleStates(speeds);
    drive(module_states);
}


void Drivetrain::drive(wpi::array<frc::SwerveModuleState, 4> const& module_states)
{
    std::vector<std::thread> t;
    unsigned                 wheel_idx = 0;
    for(auto&& state : module_states)
        t.emplace_back(wheels[wheel_idx++]->drive(state));
    for(auto&& ts : t)
        ts.join();
}

void Drivetrain::trajectoryDrive(frc::Trajectory::State const& state, frc::Rotation2d const& rotation)
{
    drive(controller.Calculate(m_odometry.GetPose(), state, rotation));
}

void Drivetrain::trajectoryDrive(frc::Trajectory::State&& state, frc::Rotation2d&& rotation)
{
    drive(controller.Calculate(m_odometry.GetPose(), state, rotation));
}

void Drivetrain::update_odometry()
{
    m_odometry.Update(get_heading(), wheels[0]->get_state(), wheels[1]->get_state(), wheels[2]->get_state(), wheels[3]->get_state());
}

void Drivetrain::face_direction(units::meters_per_second_t dx, units::meters_per_second_t dy, units::degree_t theta)
{
    auto const currentRotation = units::degree_t { get_angle() };

    int const  errorTheta = ((currentRotation - theta).to<int>() % 360 - 180 - 90) % 360;
    auto const rotateP    = 1.25;
    auto       pRotation  = errorTheta * rotateP;
    if(ngr::fabs(pRotation) > 35)
        pRotation = 35 * ((pRotation > 0) ? 1 : -1);
    drive(frc::ChassisSpeeds { dx, dy, units::degrees_per_second_t { pRotation } });
}

void Drivetrain::face_closest(units::meters_per_second_t dx, units::meters_per_second_t dy)
{
    auto const currentRotation = get_angle();
    auto const errorTheta      = (ngr::fabs(currentRotation) <= 90)
                                ? currentRotation
                                : currentRotation - 180;
    face_direction(dx, dy, units::degree_t { errorTheta });
}

inline static std::thread drive_thread;

bool auton_stop_flag = false;
void Drivetrain::auton_drive(units::meters_per_second_t dx, units::meters_per_second_t dy, units::degree_t direction)
{
    auton_stop_flag = true;     // stop previous thread
    if(drive_thread.joinable()) // verify drive thread was running, should only happen on first run
        drive_thread.join();    // wait for drivethread to finish
    auton_stop_flag = false;    // reset stop flag
    drive_thread    = std::thread { [dx, dy, direction]() {
        while(! auton_stop_flag && RobotState::IsAutonomousEnabled())
        {
            face_direction(dx, dy, direction);
            std::this_thread::sleep_for(20ms); // don't hog the cpu
        }
    } };
}

void Drivetrain::stop()
{
    auton_stop_flag = true;
    // note this takes advantage of the fact that
    // the thread starts with the wheel instructions
    // and therefore we dont need to wait for it to
    // finish the loop as long as it won't tell the
    // wheels to do anything
    std::this_thread::sleep_for(5ms); // make sure it has time to stop the thread
    for(auto&& wheel : wheels)
        wheel->stop();
}

void Drivetrain::goto180()
{
    auton_stop_flag = true;
    wpi::array<frc::SwerveModuleState, 4> states {
        frc::SwerveModuleState { 0_mps, frc::Rotation2d { units::degree_t { 180 } } },
        frc::SwerveModuleState { 0_mps, frc::Rotation2d { units::degree_t { 180 } } },
        frc::SwerveModuleState { 0_mps, frc::Rotation2d { units::degree_t { 180 } } },
        frc::SwerveModuleState { 0_mps, frc::Rotation2d { units::degree_t { 180 } } }
    };
    drive(states);
}

void Drivetrain::gotoZero()
{
    auton_stop_flag = true;
    wpi::array<frc::SwerveModuleState, 4> states {
        frc::SwerveModuleState { 0_mps, frc::Rotation2d { units::degree_t { 0 } } },
        frc::SwerveModuleState { 0_mps, frc::Rotation2d { units::degree_t { 0 } } },
        frc::SwerveModuleState { 0_mps, frc::Rotation2d { units::degree_t { 0 } } },
        frc::SwerveModuleState { 0_mps, frc::Rotation2d { units::degree_t { 0 } } }
    };
    drive(states);
}

void Drivetrain::PrintWheelAngle(int wheelid)
{
    wheels[wheelid]->printAngle();
}