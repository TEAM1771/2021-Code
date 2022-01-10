#ifndef __DRIVETRAIN_H__
#define __DRIVETRAIN_H__

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/trajectory/Trajectory.h>

namespace Drivetrain
{
    void                                 init();
    double                               get_angle(); // pull from rio
    frc::Rotation2d                      get_heading();
    frc::Pose2d                          getOdometryPose();
    void                                 print();
    void                                 drive(frc::ChassisSpeeds const& field_speeds);
    void                                 drive(wpi::array<frc::SwerveModuleState, 4> const& states);
    void                                 trajectoryDrive(frc::Trajectory::State const& state, frc::Rotation2d const& rotation);
    void                                 trajectoryAutonDrive(frc::Trajectory const& traj, frc::Rotation2d const& faceAngle);
    frc::SwerveDriveKinematics<4> const& get_kinematics();
    void                                 update_odometry();
    void                                 gotoZero();
    void                                 goto180();
    void                                 PrintWheelAngle(int);
    void                                 printOdometryPose();
    void                                 reset_gyro();
    void                                 face_direction(units::meters_per_second_t dx, units::meters_per_second_t dy, units::degree_t theta);
    void                                 face_closest(units::meters_per_second_t dx, units::meters_per_second_t dy);
    void                                 auton_drive(units::meters_per_second_t dx, units::meters_per_second_t dy, units::degree_t direction);
    void                                 stop();
} // namespace Drivetrain

#endif // __DRIVETRAIN_H__