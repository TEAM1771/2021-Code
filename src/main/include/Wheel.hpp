#ifndef __WHEEL_H__
#define __WHEEL_H__

#include "Constants.hpp"
#include <complex>
#include "Twist.hpp"
#include <iostream>

#include <ctre\Phoenix.h>

using namespace std::complex_literals; // makes '1i' work

// steering ratio 12.8:1
class Wheel
{
    using float_t = double;
    TalonFX driver, turner;
    CANCoder direction;

    double set_pos = 0; // degrees

    // helper values
    std::complex<float_t> const eia  = std::exp(1i * alpha);
    std::complex<float_t> const ieia = 1.0 / eia;

    struct polar_velocity
    {
        float_t speed;
        float_t direction;
    };
    polar_velocity get_vector_for(Twist_R const &twist);
    polar_velocity check_alternate_direction(polar_velocity const &angle);
public:
    float_t const alpha, beta, l, radius;
    Wheel(WHEELS::WheelInfo const &wheel_info);
    Wheel(Wheel const&) = delete;
    Wheel(Wheel &&) = delete;
    void printAngle();
    float_t get_angle();
    void drive(Twist_R const &twist);
};
#endif // __WHEEL_H__