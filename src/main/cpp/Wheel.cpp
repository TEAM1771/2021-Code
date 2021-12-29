#include "Wheel.hpp"
#include <iomanip>
#include <thread>

#include <frc/kinematics/SwerveDriveKinematics.h>

/******************************************************************/
/*                             Constants                          */
/******************************************************************/

constexpr double K_ENCODER_TICKS_PER_ROTATION = 2048;
constexpr double DRIVER_RATIO             = 1 * 8.16 * K_ENCODER_TICKS_PER_ROTATION; //Previous value was .25 instead of .1
constexpr double TURNING_RATIO            = 1;                                   //4096.0/360;//.125 * 12.8 * 2048 / 360;


/******************************************************************/
/*                      Non Static Functions                      */
/******************************************************************/
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


Wheel::float_t Wheel::getAngle()
{
    return direction.GetAbsolutePosition(); // return turner encoder converted to radians
}

std::thread Wheel::drive(frc::SwerveModuleState const& state)
{
    return std::thread { [this, state] {
        auto const current_rotation = frc::Rotation2d(units::degree_t(direction.GetAbsolutePosition()));
        auto const [speed, angle]   = frc::SwerveModuleState::Optimize(
            state,
            current_rotation);

        // Find the difference between our current rotational position + our new rotational position
        frc::Rotation2d rotation_delta = angle - current_rotation;

        // Find the new absolute position of the module based on the difference in rotation
        double const delta_ticks = (rotation_delta.Degrees().to<double>() / 360) * K_ENCODER_TICKS_PER_ROTATION;
        // Convert the CANCoder from it's position reading back to ticks
        double const current_ticks = direction.GetPosition() / .0878;
        double const desired_ticks = current_ticks + delta_ticks;

        //  double const velocity = speed.to<double>();

        driver.Set(ControlMode::Velocity, speed.to<double>() * radius.to<double>() * DRIVER_RATIO);
        turner.Set(ControlMode::Position, desired_ticks);
        if(id == 0)
        {
            printf("v: %5.0f\tr: %5.0f\tdt: %5.0f\tc: %5.0f\ta: %5i\tc: %5i\tdir: %5.0f\trad: %f\n",
                   speed.to<double>() * radius.to<double>() * DRIVER_RATIO,
                   desired_ticks,
                   delta_ticks,
                   current_ticks,
                   static_cast<int>(angle.Degrees()) / 360,
                   static_cast<int>(current_rotation.Degrees()) / 360,
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
    std::cout << "Angle: " << getAngle() << std::endl;
}