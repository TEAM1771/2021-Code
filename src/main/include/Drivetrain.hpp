#ifndef __DRIVETRAIN_H__
#define __DRIVETRAIN_H__

#include "Wheel.hpp"
#include <AHRS.h>
#include <array>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <iostream>
#include <memory>
#include <thread>

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
} // namespace Drivetrain

#endif // __DRIVETRAIN_H__