#pragma once

#include "JoystickButton.hpp"
#include <frc/Joystick.h>

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