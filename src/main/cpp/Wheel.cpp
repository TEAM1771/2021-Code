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
    // set_pos = direction.GetAbsolutePosition();

    turner.SetNeutralMode(NeutralMode::Coast);
    turner.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, 0, 10);
    // turner.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0,  10);
    turner.ConfigRemoteFeedbackFilter(static_cast<int>(cancoder_adr), RemoteSensorSource::RemoteSensorSource_CANCoder, 0);
    turner.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
    turner.SelectProfileSlot(0, 0);
    turner.Config_kF(0, 0.3, 10);
    turner.Config_kP(0, 1.5, 10);
    turner.Config_kI(0, 0.0, 10);
    turner.Config_kD(0, 0.3, 10);

    // turner.SetSelectedSensorPosition((direction.GetAbsolutePosition()-beta_offset)*WHEELS::turning_ratio, 0, 10);


    driver.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
    driver.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
    driver.SelectProfileSlot(0, 0);
    driver.Config_kF(0, 0.3, 10);
    driver.Config_kP(0, 0.1, 10);
    driver.Config_kI(0, 0.0, 10);
    driver.Config_kD(0, 0.0, 10);
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