#include "Wheel.hpp"
#include <iomanip>

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
    turner_config.slot0.kD                           = .3;
    turner_config.slot0.kF                           = .3;
    turner_config.remoteFilter0.remoteSensorDeviceID = direction.GetDeviceNumber();
    turner_config.remoteFilter0.remoteSensorSource   = RemoteSensorSource::RemoteSensorSource_CANCoder;
    turner_config.primaryPID.selectedFeedbackSensor  = FeedbackDevice::RemoteSensor0;
    turner.ConfigAllSettings(turner_config);

    TalonFXConfiguration driver_config {};
    driver_config.slot0.kP = 0.1;
    driver_config.slot0.kI = 0;
    driver_config.slot0.kD = 0;
    driver_config.slot0.kF = .3;
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

void Wheel::drive(frc::SwerveModuleState const& state)
{
    auto const [speed, angle] = frc::SwerveModuleState::Optimize(
        state,
        units::radian_t(direction.GetAbsolutePosition()));
    driver.Set(ControlMode::Velocity, speed.to<double>() / radius.to<double>() * WHEELS::driver_ratio);
    turner.Set(ControlMode::Position, frc::Rotation2d(angle).Degrees().to<double>() * WHEELS::turning_ratio);
}
void Wheel::printAngle()
{
    std::cout << "Angle: " << get_angle() << std::endl;
}