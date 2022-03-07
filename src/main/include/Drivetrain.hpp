#pragma once

#include <frc/kinematics/SwerveDriveKinematics.h>

namespace Drivetrain
{
    void   init();
    double getAngle(); // pull from rio
    void   print();
    void   drive(frc::ChassisSpeeds const& feild_speeds);
    void   drive(wpi::array<frc::SwerveModuleState, 4> const& states);
    void   goToZero();
    void   goTo180();
    void   printWheelAngle(int);
    void   resetGyro();
    void   faceDirection(units::meters_per_second_t dx, units::meters_per_second_t dy, units::degree_t theta);
    void   faceClosest(units::meters_per_second_t dx, units::meters_per_second_t dy);
    
    void autonDrive(units::meters_per_second_t dx, units::meters_per_second_t dy, units::degree_t direction);
    void stop();
} // namespace Drivetrain