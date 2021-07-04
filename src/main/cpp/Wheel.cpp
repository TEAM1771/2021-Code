#include "Wheel.hpp"
#include <iomanip>
#include <thread>

#include <frc/kinematics/SwerveDriveKinematics.h>
Wheel::Wheel(WHEELS::WheelInfo const& wheel_info)
    : driver { wheel_info.driver }
    , turner { wheel_info.turner }
    , direction { wheel_info.cancoder }
    , cancoder_adr { wheel_info.cancoder }
    , wheel_pos { wheel_info.wheel_pos }
{
    direction.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);

    TalonFXConfiguration turner_config {};
    turner_config.slot0.kP                           = 1.5;
    turner_config.slot0.kI                           = 0;
    turner_config.slot0.kD                           = .4;
    turner_config.slot0.kF                           = 0;
    turner_config.remoteFilter0.remoteSensorDeviceID = direction.GetDeviceNumber();
    turner_config.remoteFilter0.remoteSensorSource   = RemoteSensorSource::RemoteSensorSource_CANCoder;
    turner_config.primaryPID.selectedFeedbackSensor  = FeedbackDevice::RemoteSensor0;
    turner.ConfigAllSettings(turner_config);

    TalonFXConfiguration driver_config {};
    driver_config.slot0.kP = 1;
    driver_config.slot0.kI = 0;
    driver_config.slot0.kD = 0;
    driver_config.slot0.kF = 0;
    turner.ConfigAllSettings(turner_config);

    CANCoderConfiguration direction_config {};
    direction_config.magnetOffsetDegrees    = wheel_info.offset.to<double>();
    direction_config.initializationStrategy = SensorInitializationStrategy::BootToAbsolutePosition;
    direction.ConfigAllSettings(direction_config);
}


Wheel::float_t Wheel::get_angle()
{
    return direction.GetAbsolutePosition(); // return turner encoder converted to radians
}

std::thread Wheel::drive(frc::SwerveModuleState const& state)
{
    return std::thread { [this, state] {
        auto const currentRotation = frc::Rotation2d(units::degree_t(direction.GetAbsolutePosition()));
        auto const [speed, angle]  = frc::SwerveModuleState::Optimize(
            state,
            currentRotation);

        // Find the difference between our current rotational position + our new rotational position
        frc::Rotation2d rotationDelta = angle - currentRotation;

        // Find the new absolute position of the module based on the difference in rotation
        double const deltaTicks = (rotationDelta.Degrees().to<double>() / 360) * WHEELS::kEncoderTicksPerRotation;
        // Convert the CANCoder from it's position reading back to ticks
        double const currentTicks = direction.GetPosition() / .0878;
        double const desiredTicks = currentTicks + deltaTicks;

        double const velocity = speed.to<double>();
        if(id == 0)
        {
            // printf("speed: %f\n", velocity);
        }

        driver.Set(ControlMode::PercentOutput, velocity);
        turner.Set(ControlMode::Position, desiredTicks);
    } };
}
void Wheel::printAngle()
{
    std::cout << "Angle: " << get_angle() << std::endl;
}