#pragma once

#include "JoystickButton.hpp"
#include "Limelight.hpp"
#include "ngr.hpp"
#include <cmath>
#include <ctre/Phoenix.h>
#include <frc/Joystick.h>
#include <rev/CANSparkMax.h>
using can_adr = unsigned;


namespace BUTTON
{
    inline frc::Joystick rStick { 0 },
        lStick { 1 },
        oStick { 2 };
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
    } // namespace SHOOTER

    namespace CLIMBER
    {
        inline JoystickButton RAISE { BUTTON::oStick, 11 };
    }
} // namespace BUTTON

namespace WHEELS
{
    struct WheelInfo
    {
        // stored in radians and inches
        int    driver, turner, cancoder;
        double alpha, beta, l, radius;

        // inputs in degrees and inches
        constexpr WheelInfo(int d, int t, int c, double a_d, double b_d, double l_in, double d_in)
            : driver { d }
            , turner { t }
            , cancoder { c }
            , alpha { ngr::deg2rad(a_d) }
            , beta { ngr::deg2rad(b_d) }
            , l { l_in }
            , radius { d_in / 2 }
        {}
    };
    constexpr WheelInfo WHEEL_1 { 30, 31, 32, 45, -45-262.881, 15.573, 2 };
    constexpr WheelInfo WHEEL_2 { 40, 41, 42, 135, -135-161.895, 15.573, 2 };
    constexpr WheelInfo WHEEL_3 { 50, 51, 52, -135, 135-269.121, 15.573, 2 };
    constexpr WheelInfo WHEEL_4 { 60, 61, 62, -45, 45-287.227, 15.573, 2 };

    constexpr double turning_ratio = 12.8 * 360 / 2048;
} // namespace WHEELS

namespace CAMERA
{
    constexpr double X_OFFSET = 4.2517710;
} // namespace CAMERA

namespace HOOD
{
    constexpr can_adr PORT = 7;

    constexpr auto IDLE_MODE = rev::CANSparkMax::IdleMode::kBrake;

    constexpr double P = 0.1;
    constexpr double I = 0.0;
    constexpr double D = 0.0;

    constexpr double MAX_SPEED = 0.8;

    typedef enum {
        BOTTOM       = 0,
        TRAVERSE     = -9,
        SAFE_TO_TURN = -42,
        MIDPOINT     = -26,
        BATTER       = -89
    } POSITION;

    constexpr double TOLERANCE = 1;
} // namespace HOOD

namespace TURRET
{
    constexpr can_adr PORT = 6;

    constexpr auto IDLE_MODE = rev::CANSparkMax::IdleMode::kCoast;

    typedef enum {
        MAX_LEFT                  = -74,
        FRONT                     = -53,
        SAFE_TO_DEPLOY_HOOD_FRONT = -44,
        ZERO                      = 0,
        BACK                      = 53,
        MAX_RIGHT                 = 74
    } POSITION;

    constexpr double TOLERANCE = 10;

    constexpr double TICKS_PER_REVOLUTION = 212; // replace me with correct, number. this should be close if not exact
    constexpr double TICKS_PER_RADIAN     = TICKS_PER_REVOLUTION / (2 * pi);

    constexpr double TRAVERSE_SPEED = .7;

    constexpr double P = 0.1;
    constexpr double I = 0.0;
    constexpr double D = 0.0;

} // namespace TURRET

namespace SHOOTER_WHEEL
{
    constexpr can_adr PORT_1 = 5;
    constexpr auto IDLE_MODE = rev::CANSparkMax::IdleMode::kCoast;
    constexpr double SHOOTING_RPM = 8000;
} // namespace SHOOTER_WHEEL

namespace AUTO
{
    namespace THREE_BALL
    {
        using namespace std::literals::chrono_literals;

        constexpr double drive_distance     = 2;
        constexpr auto   minimum_shoot_time = 10s;
    } // namespace THREE_BALL

    namespace FIVE_BALL
    {
        using namespace std::literals::chrono_literals;

        constexpr double PICKUP_DISTANCE = 115;
        constexpr auto   TURN_TIME       = 0.2s;
        constexpr auto   TIME_BACKWARD   = 1.5s;
    } // namespace FIVE_BALL
} // namespace AUTO
namespace HOPPER
{
    namespace INDEXER
    {
        constexpr can_adr PORT  = 10;
        constexpr double  SPEED = 1;

        constexpr auto IDLE_MODE = rev::CANSparkMax::IdleMode::kBrake;
    } // namespace INDEXER

    namespace TRANSPORT
    {
        constexpr can_adr PORT = 3;

        constexpr auto IDLE_MODE = rev::CANSparkMax::IdleMode::kBrake;

        constexpr double SPEED       = 0.7;
        constexpr double SHOOT_SPEED = 1.0;

        constexpr double DISTANCE  = 73.0 / 3;
        constexpr double TOLERANCE = 1;

        constexpr double P = 0.3;
        constexpr double I = 0;
        constexpr double D = 0.0001;
    } // namespace TRANSPORT

    constexpr can_adr LIMIT_SWITCH = 0;

} // namespace HOPPER

namespace CLIMBER
{
    constexpr int PORT_1 = 47;
    constexpr int PORT_2 = 9;

    constexpr auto IDLE_MODE = rev::CANSparkMax::IdleMode::kBrake;

    constexpr double P = 0.1771;
    constexpr double I = 0.0;
    constexpr double D = 0.0;

    constexpr double MAX_OUTPUT = 1;

    typedef enum {
        DOWN = 100,
        UP   = 802,
        ZERO = 0
    } POSITION;


} // namespace CLIMBER

namespace INTAKE
{
    constexpr can_adr PCM_PORT = 1;
    constexpr can_adr PORT     = 12;

    constexpr auto IDLE_MODE = rev::CANSparkMax::IdleMode::kCoast;

    enum class DIRECTION {
        OUT,
        OFF,
        IN
    };

    constexpr double IN_SPEED  = -1;
    constexpr double OUT_SPEED = 1;
} // namespace INTAKE
