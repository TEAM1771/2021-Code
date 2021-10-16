#include "Turret.hpp"
#include <cmath>
// #include "PhotonVision.hpp"
#include "PhotonLib/PhotonCamera.hpp"


#include "PID_CANSparkMax.hpp"

// extern photonlib::PhotonCamera camera; //camera from Robot.cpp

inline static PID_CANSparkMax  turretTurnyTurny { TURRET::PORT, rev::CANSparkMaxLowLevel::MotorType::kBrushless };
inline static TURRET::POSITION position = TURRET::POSITION::ZERO;
inline static bool             tracking = false;

/******************************************************************/
/*                      Non Static Functions                      */
/******************************************************************/

void Turret::init()
{
    turretTurnyTurny.RestoreFactoryDefaults();

    turretTurnyTurny.SetIdleMode(TURRET::IDLE_MODE);

    turretTurnyTurny.SetP(TURRET::P);
    turretTurnyTurny.SetI(TURRET::I);
    turretTurnyTurny.SetD(TURRET::D);

    turretTurnyTurny.SetOutputRange(-TURRET::TRAVERSE_SPEED, TURRET::TRAVERSE_SPEED);
    turretTurnyTurny.SetPositionRange(TURRET::POSITION::MAX_LEFT, TURRET::POSITION::MAX_RIGHT);
    turretTurnyTurny.SetTarget(TURRET::POSITION::ZERO);
}

bool Turret::goToPosition(TURRET::POSITION pos, double tolerance)
{
    if(pos != position)
    {
        turretTurnyTurny.SetTarget(pos);
        position = pos;
    }

    tracking = false; // Reset for Turret::visionTrack(...)

    return std::fabs(turretTurnyTurny.encoder.GetPosition() - pos) < tolerance;
}

// Turret::visionState Turret::visionTrack_v1(TURRET::POSITION initPosition, double tolerance)
// {
//     if(! tracking) // move to initPosition
//     {
//         tracking = goToPosition(initPosition);
//         return { false, false };
//     }

//     if(camera.hasTarget())
//     {
//         double const xOffset = camera.getX() + CAMERA::X_OFFSET;
//         double const output  = xOffset / 35;
//         turretTurnyTurny.Set(output);
//         return { true, fabs(xOffset) < tolerance };
//     }
//     turretTurnyTurny.Set(0);
//     return { false, false };
// }

Turret::visionState Turret::visionTrack(TURRET::POSITION initPosition, double tolerance)
{
    if(! tracking) // move to initPosition
    {
        tracking = goToPosition(initPosition);
        return { false, false };
    }
    int err = 0;

    // printf("hasTarget: %i,x:%i,y:%i\n", camera.hasTarget(),camera.getX(),camera.getY());
    std::cerr << err++ << " Turret ERROR\n";
    
    photonlib::PhotonPipelineResult result = camera.GetLatestResult();
    
    std::cerr << err++ << " Turret ERROR\n";

    if(result.HasTargets())
    {
            std::cerr << err++ << " Turret ERROR\n";

        auto const target = result.GetBestTarget();
            std::cerr << err++ << " Turret ERROR\n";
        double const xOffsetDeg = target.GetYaw() + CAMERA::X_OFFSET;
            std::cerr << err++ << " Turret ERROR\n";
        double const xOffsetRad = ngr::deg2rad(xOffsetDeg);
        double const xOffset    = xOffsetRad * TURRET::TICKS_PER_RADIAN;

        double const xPosition = turretTurnyTurny.encoder.GetPosition();
        double const xTarget   = xPosition + xOffset;

        static double prevOffsetDeg = 0;
        if(prevOffsetDeg == xOffsetDeg) // prevents reusing outdated data
            return { true, fabs(xOffsetDeg) < tolerance };
        prevOffsetDeg = xOffsetDeg;

        turretTurnyTurny.SetTarget(xTarget);

        return { true, fabs(xOffsetDeg) < tolerance };
    }

    // return { true, true };
    return { false, false };
}

void Turret::manualPositionControl(double pos)
{
    turretTurnyTurny.SetTarget(ngr::scaleOutput(
                                   -1,
                                   1,
                                   TURRET::POSITION::MAX_LEFT,
                                   TURRET::POSITION::MAX_RIGHT,
                                   std::clamp(pos, -1.0, 1.0)),
                               rev::ControlType::kPosition);
}
