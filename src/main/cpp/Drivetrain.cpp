#include "drivetrain.hpp"
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>

#include <thread>
// #include <cmath>
Drivetrain::Drivetrain()
{
    navx->ZeroYaw();
}


double Drivetrain::get_angle()
{
    return navx->GetAngle();
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
    unsigned wheel_idx = 0;
    for(auto&& state : module_states)
        wheels[wheel_idx++]->drive(state);
}

void Drivetrain::goto180()
{
}

void Drivetrain::gotoZero()
{
}
