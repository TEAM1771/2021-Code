#ifndef __DRIVETRAIN_H__
#define __DRIVETRAIN_H__

#include <frc/kinematics/SwerveDriveKinematics.h>

namespace Drivetrain
{
    void   init();
    double get_angle(); // pull from rio
    void   print();
    void   drive(frc::ChassisSpeeds const& feild_speeds);
    void   drive(wpi::array<frc::SwerveModuleState, 4> const& states);
    void   gotoZero();
    void   goto180();
    void   PrintWheelAngle(int);
    void   reset_gyro();
    void   face_direction(units::meters_per_second_t dx, units::meters_per_second_t dy, units::degree_t theta);
    void   face_closest(units::meters_per_second_t dx, units::meters_per_second_t dy);
} // namespace Drivetrain

#endif // __DRIVETRAIN_H__