#include "Hood.hpp"
//#include "PhotonVision.hpp"
#include "LimeLight.hpp"
//#include "Average.hpp"
#include <algorithm>
#include <cmath>
#include <vector>
#include <PID_CANSparkMax.hpp>
// #include <PhotonCamera.h> I think I included the right file above

extern LimeLight camera; // From Robot.cpp
//Average<3>   averageInputCameraY;

static inline PID_CANSparkMax hood { HOOD::PORT, rev::CANSparkMaxLowLevel::MotorType::kBrushless };
static inline HOOD::POSITION  position = HOOD::POSITION::BOTTOM;

/******************************************************************/
/*                      Non Static Functions                      */
/******************************************************************/

void Hood::init()
{
    hood.RestoreFactoryDefaults();
    hood.SetIdleMode(HOOD::IDLE_MODE);
    hood.SetSmartCurrentLimit(20);

    hood.SetP(HOOD::P);
    hood.SetI(HOOD::I);
    hood.SetD(HOOD::D);

    hood.SetTarget(HOOD::POSITION::BOTTOM);
    hood.SetOutputRange(-HOOD::MAX_SPEED, HOOD::MAX_SPEED);
    hood.SetPositionRange(HOOD::POSITION::BATTER, HOOD::POSITION::BOTTOM);
}

bool Hood::goToPosition(HOOD::POSITION pos, double tolerance)
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

    if (yval < -3.6) {
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

    static_assert(ngr::is_close_to(std::midpoint(lookup_table[0].hood_val, lookup_table[1].hood_val), interpolate(std::midpoint(lookup_table[0].y_val, lookup_table[1].y_val), &lookup_table[0], &lookup_table[1])),
                  "interpolation error");
}

bool Hood::visionTrack(double tolerance)
{
    // auto const result       = camera.GetLatestResult();
    if(camera.hasTarget())
    {
        // auto const cameratarget = result.GetBestTarget();
        double target = getTrackingValue(camera.getY());
        hood.SetTarget(std::clamp(target, static_cast<double>(HOOD::SAFE_TO_TURN), 0.0));
        return std::fabs(target - hood.encoder.GetPosition()) < tolerance;
    }
    goToPosition(HOOD::POSITION::TRAVERSE);
    return false;
}

void Hood::manualPositionControl(double pos)
{
    hood.SetTarget(ngr::scaleOutput(0,
                                    1,
                                    HOOD::POSITION::TRAVERSE,
                                    HOOD::POSITION::SAFE_TO_TURN,
                                    std::clamp(pos, 0.0, 1.0)));
}

void Hood::print_angle()
{
    printf("hood angle: %f\n", hood.encoder.GetPosition());
}

double Hood::get_angle()
{
    return hood.encoder.GetPosition();
}

double Hood::get_camera_Y()
{
    // auto const result       = camera.GetLatestResult();
    // auto const cameratarget = result.GetBestTarget();
    return camera.getY();
}