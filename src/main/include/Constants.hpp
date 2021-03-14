#pragma once

#include "JoystickButton.hpp"
#include "Limelight.hpp"
#include <cmath>
#include <ctre/Phoenix.h>
#include <frc/Joystick.h>
#include <rev/CANSparkMax.h>

using can_adr = unsigned;

constexpr double pi = 3.1415926;

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

namespace TRANSMISSION
{
    constexpr can_adr RIGHT_MOTOR = 15;
    constexpr can_adr LEFT_MOTOR  = 17;

    constexpr auto IDLE_MODE = NeutralMode::Coast;

    constexpr can_adr SHIFTER = 0;

    constexpr int SHIFT_UP_POINT   = 16000;
    constexpr int SHIFT_DOWN_POINT = 6000;
} // namespace TRANSMISSION

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
    constexpr can_adr PORT_1 = 4;
    constexpr can_adr PORT_2 = 5;

    constexpr auto IDLE_MODE = rev::CANSparkMax::IdleMode::kCoast;

    constexpr double SHOOTING_RPM = 8000;
} // namespace SHOOTER_WHEEL

namespace AUTO
{
    namespace THREE_BALL
    {
        using namespace std::literals::chrono_literals;

        constexpr double drive_distance     = 2;
        constexpr auto minimum_shoot_time = 10s;
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


namespace ngr // North Gwinnett Robotics
{
    [[nodiscard]] constexpr bool value_in_range(double value, double min, double max)
    {
        return value > min && value < max;
    }
    static_assert(value_in_range(1, 0, 2) == true);
    static_assert(value_in_range(0, 0, 1) == false);
    static_assert(value_in_range(1, 0, 1) == false);
    static_assert(value_in_range(2, 0, 1) == false);
    static_assert(value_in_range(-1, 0, 1) == false);

    // it compiled fine with std::fabs this, but this got rid of red squigles
    [[nodiscard]] constexpr static double fabs(double value)
    {
        if(value < 0)
            return -value;
        else
            return value;
    }
    static_assert(fabs(1) == 1);
    static_assert(fabs(-1) == 1);
    static_assert(fabs(0) == 0);

    // floating point comparison at compile time
    [[nodiscard]] constexpr static bool is_close_to(double value,
                                                    double target,
                                                    double tol = 0.00001)
    {
        return fabs(value - target) < tol;
    }
    static_assert(is_close_to(.5, .500000001) == true);
    static_assert(is_close_to(.1, .999989) == false);
    static_assert(is_close_to(.1, .10002) == false);


    [[nodiscard]] constexpr static double scaleOutput(double inputMin, double inputMax, double outputMin, double outputMax, double input)
    {
        return ((input - inputMin) / (inputMax - inputMin)) * ((outputMax - outputMin)) + outputMin;
    }
    static_assert(is_close_to(scaleOutput(0, 1, -1, 1, 0), -1));
    static_assert(is_close_to(scaleOutput(0, 1, -1, 1, 1), 1));
    static_assert(is_close_to(scaleOutput(0, 1, -1, 1, .5), 0));

    [[nodiscard]] constexpr static double deg2rad(double deg)
    {
        return deg * pi / 180;
    }
    static_assert(is_close_to(deg2rad(360), 2 * pi));
    static_assert(is_close_to(deg2rad(0), 0));

    [[nodiscard]] constexpr static double rad2deg(double rad)
    {
        return rad * 180 / pi;
    }
    static_assert(is_close_to(rad2deg(pi), 180));
    static_assert(is_close_to(rad2deg(0), 0));

} // namespace ngr


// bad, but good enough implimentation of std::midpoint from C++20
// remove this if upgraded to C++20
namespace std
{
    template <typename A, typename B>
    constexpr std::common_type_t<A, B> midpoint(A const& a, B const& b)
    {
        return (a + b) / 2;
    }
} // namespace std
