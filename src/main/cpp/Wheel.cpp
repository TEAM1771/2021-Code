#include "Wheel.hpp"
#include <iomanip>
Wheel::Wheel(WHEELS::WheelInfo const& wheel_info)
    : driver { wheel_info.driver }
    , turner { wheel_info.turner }
    , direction { wheel_info.cancoder }
    , cancoder_adr { wheel_info.cancoder }
    , alpha { wheel_info.alpha }
    , beta { wheel_info.beta + wheel_info.beta_offset }
    , beta_offset { wheel_info.beta_offset }
    , l { wheel_info.l }
    , radius { wheel_info.radius }
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

class Wheel{
using float_t = double;

TalonFX driver, turner;
CANCoder direction;
can_adr cancoder_adr;

frc::Translation2d const wheel_pos;
units::meter_t radius;
public:
Wheel(WHEELS::WheelInfo const& wheel_info);
Wheel(Wheel const&) = delete;
Wheel(Wheel&&)      = delete;

constexpr operator frc::Translation2d() const
{
    return wheel_pos;
}
void    printAngle();
float_t get_angle();
void    drive(frc::SwerveModuleState const& state);
};

void Wheel::printAngle()
{
    std::cout << "Angle: " << get_angle() << std::endl;
}
