#include "Wheel.hpp"
Wheel::Wheel(WHEELS::WheelInfo const& wheel_info)
    : driver { wheel_info.driver }
    , turner { wheel_info.turner }
    , direction { wheel_info.cancoder }
    , alpha { wheel_info.alpha }
    , beta { wheel_info.beta }
    , l { wheel_info.l }
    , radius { wheel_info.radius }
{
    set_pos = direction.GetAbsolutePosition();
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
    return ngr::deg2rad(direction.GetAbsolutePosition()); // return turner encoder converted to radians
}

void Wheel::drive(Twist_R const& twist)
{
    auto const vect = get_vector_for(twist);
    driver.Set(TalonFXControlMode::Velocity, vect.speed);

    auto dir_d = static_cast<int>(ngr::rad2deg(vect.direction)) % 360;
    if(dir_d < 0)
        dir_d += 360;

    auto const cur_d = static_cast<int>(set_pos) % 360;
    auto const diff_d = cur_d - dir_d;
    if(diff_d > 180)
        set_pos -= diff_d;
    else
        set_pos += diff_d;

    turner.Set(TalonFXControlMode::Position, set_pos * WHEELS::turning_ratio);
}

void Wheel::printAngle()
{
    std::cout << "Angle: " << get_angle() << std::endl;
}