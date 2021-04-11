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

Wheel::polar_velocity Wheel::get_vector_for(Twist_R const& twist)
{
    // get angle
    auto const itl = 1i * l * twist.dtheta;

    auto const top    = itl + (twist.dx + 1i * twist.dy) * ieia;
    auto const bottom = itl + (-twist.dx + 1i * twist.dy) * eia;

    double const angle = std::real(-1i * log(top / bottom)) / 2 + beta;

    // get velocity
    double const velocity = (twist.dx * std::sin(alpha + angle) + twist.dy * std::cos(alpha + angle) + l * twist.dtheta * std::cos(angle)) / radius;

    return { velocity, angle };
}

Wheel::float_t Wheel::get_angle()
{
    return direction.GetAbsolutePosition(); // return turner encoder converted to radians
}

void Wheel::drive(Twist_R const& twist)
{
    auto const vect = get_vector_for(twist);
    // driver.Set(TalonFXControlMode::Velocity, vect.speed * WHEELS::driver_ratio);

    auto dir_d = static_cast<int>(ngr::rad2deg(vect.direction)) % 360;
    if(dir_d < 0)
        dir_d += 360;

    // auto const cur_d  = static_cast<int>(set_pos) % 360;
    // auto const diff_d = cur_d - dir_d;
    // if(vect.speed > .5)
    // {
    //     if(diff_d > 180)
    //         set_pos -= diff_d;
    //     else
    //         set_pos += diff_d;
    // }
    set_pos = dir_d;
    turner.Set(TalonFXControlMode::Position, set_pos*WHEELS::turning_ratio);
    if(wheelid == 2)
        std::cout << std::setprecision(3) << set_pos << "   " << get_angle() << "    " << turner.GetClosedLoopTarget() << "    " << turner.GetClosedLoopError() << '\n';
}
void Wheel::printAngle()
{
    std::cout << "Angle: " << get_angle() << std::endl;
}