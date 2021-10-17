#include "drivetrain.hpp"
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>

#include <thread>
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

inline static std::unique_ptr<AHRS> navx { std::make_unique<AHRS>(frc::SPI::Port::kMXP) };


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
    return -navx->GetAngle() + 90;
}

void Drivetrain::drive(frc::ChassisSpeeds const& feild_speeds)
{
    auto const speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        feild_speeds.vx,
        feild_speeds.vy,
        feild_speeds.omega,
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

void Drivetrain::face_direction(units::meters_per_second_t dx, units::meters_per_second_t dy, units::degree_t theta)
{
    auto const currentRotation = units::degree_t { get_angle() };
    auto const errorTheta      = currentRotation - theta;
    auto const rotateP         = 1.5;
    auto slowedRotation  = errorTheta * rotateP / 1_s;
    if (slowedRotation > 90_deg / 1_s) 
        slowedRotation = 90_deg / 1_s;
    drive({ dx, dy, slowedRotation});
}

void Drivetrain::goto180()
{
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