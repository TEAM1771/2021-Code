#include "Drivetrain.hpp"
#include "Wheel.hpp"
#include "RobotState.hpp"
#include "ngr.hpp"

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Translation2d.h>
#include <AHRS.h>

#include <array>
#include <thread>

/******************************************************************/
/*                          Non-constant Vars                     */
/******************************************************************/

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

inline static std::unique_ptr<AHRS> navx { std::make_unique<AHRS>(frc::SPI::Port::kMXP) };


/******************************************************************/
/*                      Non Static Functions                      */
/******************************************************************/

using namespace std::chrono_literals;
void Drivetrain::init()
{
    resetGyro();
}

void Drivetrain::resetGyro()
{
    navx->ZeroYaw();
}

double Drivetrain::getAngle()
{
    return -navx->GetAngle() + 90;
}

void Drivetrain::drive(frc::ChassisSpeeds const& field_speeds)
{
    auto const speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        field_speeds.vx,
        field_speeds.vy,
        field_speeds.omega,
        frc::Rotation2d { units::degree_t { getAngle() } });
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


void Drivetrain::faceDirection(units::meters_per_second_t dx, units::meters_per_second_t dy, units::degree_t theta)
{
    auto const current_rotation = units::degree_t { getAngle() };

    int const error_theta      = ((current_rotation - theta).to<int>()%360-180-90)%360;
    auto const rotate_p         = 1.25;
    auto       p_rotation       = error_theta * rotate_p;
    if(ngr::fabs(p_rotation) > 35)
         p_rotation = 35 * ((p_rotation > 0)?1:-1);
    drive(frc::ChassisSpeeds{ dx, dy, units::degrees_per_second_t{p_rotation} });
}

void Drivetrain::faceClosest(units::meters_per_second_t dx, units::meters_per_second_t dy)
{
    auto const current_rotation = getAngle();
    auto const error_theta      = (ngr::fabs(current_rotation) <= 90)
                                ? current_rotation
                                : current_rotation - 180;
    faceDirection(dx, dy, units::degree_t {error_theta});
}

inline static std::thread drive_thread;

bool auton_stop_flag = false;
void Drivetrain::autonDrive(units::meters_per_second_t dx, units::meters_per_second_t dy, units::degree_t direction)
{
    auton_stop_flag = true;     // stop previous thread
    if(drive_thread.joinable()) // verify drive thread was running, should only happen on first run
        drive_thread.join();    // wait for drivethread to finish
    auton_stop_flag = false;    // reset stop flag
    drive_thread    = std::thread { [dx, dy, direction]() {
        while(! auton_stop_flag && RobotState::isAutonomousEnabled())
        {
            faceDirection(dx, dy, direction);
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

void Drivetrain::goTo180()
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

void Drivetrain::goToZero()
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

void Drivetrain::printWheelAngle(int wheel_id)
{
    wheels[wheel_id]->printAngle();
}