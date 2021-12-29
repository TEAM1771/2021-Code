#pragma once

#include "JoystickButton.hpp"
//#include "Limelight.hpp"
#include "ngr.hpp"
#include <cmath>
#include <ctre/Phoenix.h>
#include <frc/Joystick.h>
#include <rev/CANSparkMax.h>

#include <frc/geometry/Transform2d.h>

using namespace std::literals::chrono_literals;

using can_adr = int;

namespace BUTTON
{
    inline frc::Joystick ps5 { 0 },
        oStick { 1 };

    inline JoystickButton RUMBLE { BUTTON::ps5, 5 };

    namespace INTAKE
    {
        inline JoystickButton DEPLOY { BUTTON::oStick, 3 };
        inline JoystickButton RETRACT { BUTTON::oStick, 4 };
        inline JoystickButton INTAKE { BUTTON::oStick, 5 };
    } // namespace INTAKE
    namespace SHOOTER
    {
        inline JoystickButton AIM_FRONT { BUTTON::oStick, 8 };
        inline JoystickButton AIM_BACK { BUTTON::oStick, 10 };
        inline JoystickButton AIM_SIDE { BUTTON::oStick, 2 };
        inline JoystickButton BATTERSHOT { BUTTON::oStick, 6 };
        inline JoystickButton SHOOT { BUTTON::oStick, 1 };
        inline JoystickButton ADJUST_SHOOTER_UP { BUTTON::oStick, 12 };
        inline JoystickButton ADJUST_SHOOTER_DOWN { BUTTON::oStick, 11 };

    } // namespace SHOOTER

    namespace CLIMBER
    {
        inline JoystickButton RAISE { BUTTON::oStick, 11 };
    }

    namespace DRIVETRAIN
    {
        // inline JoystickButton ZERO { BUTTON::lStick, 10 };
        // inline JoystickButton REVERSE { BUTTON::lStick, 11 };
        inline JoystickButton ROTATE_FRONT { BUTTON::ps5, 7 };
        //    inline JoystickButton ROTATE_BACK { BUTTON::ps5, 7 };
        //    inline JoystickButton ROTATE_TO_CLOSEST { BUTTON::ps5, 8 };
        inline JoystickButton ROTATE_CLIMB { BUTTON::ps5, 8 };
    } // namespace DRIVETRAIN
} // namespace BUTTON

namespace WHEELS
{
    struct WheelInfo
    {
        // stored in radians and inches
        can_adr const            driver, turner, cancoder;
        frc::Translation2d const wheel_pos;
        units::inch_t const      radius;
        units::degree_t const    offset;

        // inputs in degrees and inches
        constexpr WheelInfo(can_adr d, can_adr t, can_adr c, frc::Translation2d wheel_position, units::inch_t diameter_, units::degree_t offset_)
            : driver { d }
            , turner { t }
            , cancoder { c }
            , wheel_pos { wheel_position }
            , radius { diameter_ / 2 }
            , offset { offset_ }
        {}
    };
    WheelInfo const WHEEL_1 { 30, 31, 11, { 11_in, -11_in }, 4_in, 0_deg };  // 360-275
    WheelInfo const WHEEL_2 { 40, 41, 12, { 11_in, 11_in }, 4_in, 0_deg };   // 15
    WheelInfo const WHEEL_3 { 50, 51, 13, { -11_in, 11_in }, 4_in, 0_deg };  // 360-83.9
    WheelInfo const WHEEL_4 { 60, 61, 14, { -11_in, -11_in }, 4_in, 0_deg }; // 75

    constexpr double speed_mult = 1; // hacky way to deal with joysticks
} // namespace WHEELS