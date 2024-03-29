#include "Hood.hpp"
#include "LimeLight.hpp"
#include "ngr.hpp"
// #include "Average.hpp"

#include <PID_CANSparkMax.hpp>

extern LimeLight camera; // From Robot.cpp

using can_adr = int;
/******************************************************************/
/*                             Constants                          */
/******************************************************************/
const can_adr PORT = 7;

const auto IDLE_MODE = rev::CANSparkMax::IdleMode::kBrake;

const double P = 0.1;
const double I = 0.0;
const double D = 0.0;

const double MAX_SPEED = 0.8;

// Average<10>   averageInputCameraY;

/******************************************************************/
/*                          Non-constant Vars                     */
/******************************************************************/

static inline PID_CANSparkMax hood { PORT, rev::CANSparkMaxLowLevel::MotorType::kBrushless };
static inline Hood::POSITION  position = Hood::POSITION::BOTTOM;

/******************************************************************/
/*                      Non Static Functions                      */
/******************************************************************/

void Hood::init()
{
    hood.RestoreFactoryDefaults();
    hood.SetIdleMode(IDLE_MODE);
    hood.SetSmartCurrentLimit(20);

    hood.SetP(P);
    hood.SetI(I);
    hood.SetD(D);

    hood.SetTarget(Hood::POSITION::BOTTOM);
    hood.SetOutputRange(-MAX_SPEED, MAX_SPEED);
    hood.SetPositionRange(Hood::POSITION::BATTER, Hood::POSITION::BOTTOM);
}

bool Hood::goToPosition(Hood::POSITION pos, double tolerance)
{
    if(pos != position)
    {
        hood.SetTarget(pos);
        position = pos;
    }
    return std::fabs(hood.encoder.GetPosition() - pos) < tolerance;
}

[[nodiscard]] inline static double getTrackingValue(double yval)
{
    struct table_row
    {
        double y_val;
        double hood_val;
    };

    constexpr table_row lookup_table[] {
        { 16.1, -17.023779 },
        { 10.8, -18.190428 },
        { 5.6, -19.476120 },
        { 1.95, -20.190395 },
        { -0.9, -20.809433 },
        { -3.6, -21.118952 }
    };

    if(yval < -3.6)
    {
        yval = -3.6;
    }

    auto find_value_in_table = [](auto yval, auto begin, auto end) {
        return std::find_if(std::next(begin), end, [=](auto const& val) {
            return yval >= val.y_val;
        });
    };

    constexpr auto interpolate = [](auto value, table_row const* ref1, table_row const* ref2) {
        return ((ref1->hood_val - ref2->hood_val) / (ref1->y_val - ref2->y_val)) * (value - ref2->y_val) + ref2->hood_val;
    };

    auto const range = find_value_in_table(yval, std::begin(lookup_table), std::end(lookup_table));
    return interpolate(yval,
                       std::prev(range),
                       range);

    //tests
    static_assert(std::end(lookup_table) - std::begin(lookup_table) >= 2, "lookup table too small");
    // Commented tests are valid C++20
    // static_assert(std::is_sorted(std::begin(lookup_table), std::end(lookup_table), [](auto const &lhs, auto const &rhs) {
    //                   return lhs.y_val > rhs.y_val;
    //               }),
    //               "Lookup table not sorted");

    // static_assert(
    //     find_value_in_table(lookup_table[0].y_val, std::begin(lookup_table), std::end(lookup_table))->y_val ==
    //         lookup_table[1].y_val,
    //     "Invalid Table Search");

    static_assert(ngr::isCloseTo(std::midpoint(lookup_table[0].hood_val, lookup_table[1].hood_val), interpolate(std::midpoint(lookup_table[0].y_val, lookup_table[1].y_val), &lookup_table[0], &lookup_table[1])),
                  "interpolation error");
}

bool Hood::visionTrack(double tolerance)
{
    // auto const result       = camera.GetLatestResult();
    if(camera.hasTarget())
    {
        // auto const cameratarget = result.GetBestTarget();
        double target = getTrackingValue(camera.getY());
        hood.SetTarget(std::clamp(target, static_cast<double>(Hood::POSITION::SAFE_TO_TURN), 0.0));
        return std::fabs(target - hood.encoder.GetPosition()) < tolerance;
    }
    goToPosition(Hood::POSITION::TRAVERSE);
    return false;
}

void Hood::manualPositionControl(double pos)
{
    hood.SetTarget(ngr::scaleOutput(0,
                                    1,
                                    Hood::POSITION::TRAVERSE,
                                    Hood::POSITION::SAFE_TO_TURN,
                                    std::clamp(pos, 0.0, 1.0)));
}

void Hood::printAngle()
{
    printf("hood angle: %f\n", hood.encoder.GetPosition());
}

double Hood::getAngle()
{
    return hood.encoder.GetPosition();
}

double Hood::getCameraY()
{
    // auto const result       = camera.GetLatestResult();
    // auto const cameratarget = result.GetBestTarget();
    return camera.getY();
}