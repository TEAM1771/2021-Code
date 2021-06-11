#include "drivetrain.hpp"
#include <thread>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/ChassisSpeeds.h>
// #include <cmath>

// Locations for the swerve drive modules relative to the robot center.
frc::Translation2d m_frontLeftLocation{0.381_m, 0.381_m};
frc::Translation2d m_frontRightLocation{0.381_m, -0.381_m};
frc::Translation2d m_backLeftLocation{-0.381_m, 0.381_m};
frc::Translation2d m_backRightLocation{-0.381_m, -0.381_m};

// Creating my kinematics object using the module locations.
frc::SwerveDriveKinematics<4> m_kinematics{
  m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
  m_backRightLocation};

// Example chassis speeds: 1 meter per second forward, 3 meters
// per second to the left, and rotation at 1.5 radians per second
// counterclockwise.
frc::ChassisSpeeds speeds{1_mps, 3_mps, 1.5_rad_per_s};

// Convert to module states. Here, we can use C++17's structured
// bindings feature to automatically split up the array into its
// individual SwerveModuleState components.
auto [fl, fr, bl, br] = m_kinematics.ToSwerveModuleStates(speeds);

auto flOptimized = frc::SwerveModuleState::Optimize(fl, units::radian_t(m_turningEncoder.GetDistance()));

// The desired field relative speed here is 2 meters per second
// toward the opponent's alliance station wall, and 2 meters per
// second toward the left field boundary. The desired rotation
// is a quarter of a rotation per second counterclockwise. The current
// robot angle is 45 degrees.
frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
  2_mps, 2_mps, units::radians_per_second_t(wpi::math::pi / 2.0), frc::Rotation2d(45_deg));

// Now use this in our kinematics
auto [fl, fr, bl, br] = m_kinematics.ToSwerveModuleStates(speeds);

// Example module States
frc::SwerveModuleState frontLeftState{23.43_mps, frc::Rotation2d(-140.19_deg)};
frc::SwerveModuleState frontRightState{23.43_mps, frc::Rotation2d(-39.81_deg)};
frc::SwerveModuleState backLeftState{54.08_mps, frc::Rotation2d(-109.44_deg)};
frc::SwerveModuleState backRightState{54.08_mps, frc::Rotation2d(-70.56_deg)};

// Convert to chassis speeds. Here, we can use C++17's structured bindings
// feature to automatically break up the ChassisSpeeds struct into its
// three components.
auto [forward, sideways, angular] = m_kinematics.ToChassisSpeeds(
  frontLeftState, frontRightState, backLeftState, backRightState);

// Example module States
frc::SwerveModuleState frontLeftState{23.43_mps, frc::Rotation2d(-140.19_deg)};
frc::SwerveModuleState frontRightState{23.43_mps, frc::Rotation2d(-39.81_deg)};
frc::SwerveModuleState backLeftState{54.08_mps, frc::Rotation2d(-109.44_deg)};
frc::SwerveModuleState backRightState{54.08_mps, frc::Rotation2d(-70.56_deg)};

// Convert to chassis speeds. Here, we can use C++17's structured bindings
// feature to automatically break up the ChassisSpeeds struct into its
// three components.
auto [forward, sideways, angular] = m_kinematics.ToChassisSpeeds(
  frontLeftState, frontRightState, backLeftState, backRightState);
