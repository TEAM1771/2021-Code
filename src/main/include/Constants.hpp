#pragma once

#include "JoystickButton.hpp"
#include "Limelight.hpp"
#include "ngr.hpp"
#include <cmath>
#include <ctre/Phoenix.h>
#include <frc/Joystick.h>
#include <rev/CANSparkMax.h>

#include <frc/geometry/Transform2d.h>

using can_adr = int;


namespace BUTTON
{
    inline frc::Joystick ps5 { 0 },
        oStick { 1 };
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

    namespace DRIVETRAIN
    {
        // inline JoystickButton ZERO { BUTTON::lStick, 10 };
        // inline JoystickButton REVERSE { BUTTON::lStick, 11 };
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


    constexpr double kEncoderTicksPerRotation = 4096;
    constexpr double driver_ratio             = .1 * 8.16 * 2048; //Previous value was .25 instead of .1
    constexpr double turning_ratio            = 1; //4096.0/360;//.125 * 12.8 * 2048 / 360;

    constexpr double speed_mult = 1; // hacky way to deal with joysticks
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
        SAFE_TO_TURN = -38,
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
    constexpr can_adr PORT_1       = 18;
    constexpr auto    IDLE_MODE    = rev::CANSparkMax::IdleMode::kCoast;
    constexpr double  SHOOTING_RPM = 6750;
} // namespace SHOOTER_WHEEL

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
    constexpr int PORT_1 = 9;
    constexpr int PORT_2 = 35;

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
    constexpr can_adr PORT     = 22;

    constexpr auto IDLE_MODE = rev::CANSparkMax::IdleMode::kCoast;

    enum class DIRECTION {
        OUT,
        OFF,
        IN
    };

    constexpr double IN_SPEED  = -1;
    constexpr double OUT_SPEED = 1;
} // namespace INTAKE

namespace AUTO
{
    namespace THREE_BALL
    {
        using namespace std::literals::chrono_literals;

        constexpr auto DRIVE_TIME      = 1s;
        constexpr auto SHOOT_WAIT_TIME = 1s;
        constexpr auto SPINUP_TIME     = 4.5s;

    } // namespace THREE_BALL

    namespace FIVE_BALL
    {
        using namespace std::literals::chrono_literals;

        constexpr auto MOVE_TO_BALLS     = 2.5s;
        constexpr auto PICKUP_TIME       = 1.5s;
        constexpr auto MOVE_TO_GOAL_TIME = 4.5s;
        constexpr auto SHOOT_WAIT_TIME   = 1s;


    } // namespace FIVE_BALL
    namespace SIX_BALL
    {
        using namespace std::literals::chrono_literals;
        constexpr auto SPIN_UP_TIME      = 4.5s;
        constexpr auto PICKUP_DRIVE_TIME = 2.8s;
        constexpr auto SHOOT_TIME_1      = 1s;
        constexpr auto TIME_BACKWARD     = 2.3s;
        constexpr auto SHOOT_WAIT_TIME   = 1.5s;
    } // namespace SIX_BALL

    namespace EIGHT_BALL
    {
        using namespace std::literals::chrono_literals;
        constexpr auto SHOOT_TIME_1           = 2s;
        constexpr auto TRENCH_RUN_PICKUP_TIME = 3s;
        constexpr auto TRENCH_RUN_RETURN_TIME = 2s;
        constexpr auto SHOOT_TIME_2           = 3s;
    } // namespace EIGHT_BALL

    namespace TEN_BALL
    {
        using namespace std::literals::chrono_literals;

        constexpr auto RETURN_PICKUP_TIME = 3s;
        constexpr auto PICKUP_MOVE_TIME   = 2s;
        constexpr auto PICKUP_RETURN_TIME = 2s;
        constexpr auto GOAL_RETURN_TIME   = 2s;
    } // namespace TEN_BALL

    namespace THIRTEEN_BALL
    {
        using namespace std::literals::chrono_literals;
        constexpr auto RETURN_PICKUP_TIME = 3s;
        constexpr auto PICKUP_MOVE_TIME   = 2s;
        constexpr auto PICKUP_RETURN_TIME = 2s;
        constexpr auto GOAL_RETURN_TIME   = 2s;
    } // namespace THIRTEEN_BALL
} // namespace AUTO
