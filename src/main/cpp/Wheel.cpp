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
    , radius { wheel_info.radius }
    , offset { wheel_info.offset }
{
    direction.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);

    TalonFXConfiguration turner_config {};
    turner_config.slot0.kP                           = 2.5;
    turner_config.slot0.kI                           = 0;
    turner_config.slot0.kD                           = 0;
    turner_config.slot0.kF                           = 0;
    turner_config.neutralDeadband                    = 0.001771;
    turner_config.peakOutputForward                  = .5;
    turner_config.peakOutputReverse                  = -.5;
    turner_config.remoteFilter0.remoteSensorDeviceID = direction.GetDeviceNumber();
    turner_config.remoteFilter0.remoteSensorSource   = RemoteSensorSource::RemoteSensorSource_CANCoder;
    turner_config.primaryPID.selectedFeedbackSensor  = FeedbackDevice::RemoteSensor0;
    turner_config.closedloopRamp                     = .000;
    turner.ConfigAllSettings(turner_config);

    TalonFXConfiguration driver_config {};
    driver_config.slot0.kP = .1;
    driver_config.slot0.kI = 0;
    driver_config.slot0.kD = 0;
    driver_config.slot0.kF = 0;
    // driver_config.voltageCompSaturation = 12;
    driver.ConfigAllSettings(driver_config);
    driver.SetNeutralMode(NeutralMode::Brake);
    driver.ConfigClosedloopRamp(.2);

    // CANCoderConfiguration direction_config {};
    // direction_config.magnetOffsetDegrees    = wheel_info.offset.to<double>(); // DO NOT UNCOMMENT
    // direction_config.initializationStrategy = SensorInitializationStrategy::BootToAbsolutePosition;
    // direction.ConfigAllSettings(direction_config);
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

        //  double const velocity = speed.to<double>();

        driver.Set(ControlMode::Velocity, speed.to<double>() * radius.to<double>() * WHEELS::driver_ratio);
        turner.Set(ControlMode::Position, desiredTicks);
        if(id == 0)
        {
            printf("v: %5.0f\tr: %5.0f\tdt: %5.0f\tc: %5.0f\ta: %5i\tc: %5i\tdir: %5.0f\trad: %f\n",
            speed.to<double>() * radius.to<double>() * WHEELS::driver_ratio,
            desiredTicks,
            deltaTicks,
            currentTicks,
            static_cast<int>(angle.Degrees())/360,
            static_cast<int>(currentRotation.Degrees())/360,
            direction.GetAbsolutePosition(),
            radius.to<double>());
        }
    } };
}

void Wheel::stop()
{
    driver.Set(ControlMode::Velocity, 0);
}

void Wheel::printAngle()
{
    std::cout << "Angle: " << get_angle() << std::endl;
}