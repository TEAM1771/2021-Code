#include "PhotonVision.hpp"
#include "PhotonLib/PhotonCamera.hpp"

#include <iostream>


/******************************************************************/
/*                            Constructor                         */
/******************************************************************/
PhotonCamera::PhotonCamera()
{
    camera.TakeInputSnapshot();
    setLEDMode(LED_Mode::Default);
    camera.SetPipelineIndex(2);
}

/******************************************************************/
/*                             Get Result                         */
/******************************************************************/
auto PhotonCamera::getLatestResult() const
{
    return camera.GetLatestResult();
}

/******************************************************************/
/*                           Get Target List                      */
/******************************************************************/

//From latestResult
auto PhotonCamera::getTargetList() const
{
    return getLatestResult().GetTargets();
}

//From defined result
auto PhotonCamera::getTargetList(photonlib::PhotonPipelineResult result) const
{
    return result.GetTargets();
}

/******************************************************************/
/*                           Get Best Target                      */
/******************************************************************/

//From latestResult
auto PhotonCamera::getBestTarget() const
{
    return getLatestResult().GetBestTarget();
}
//From defined result
auto PhotonCamera::getBestTarget(photonlib::PhotonPipelineResult result) const
{
    return result.GetBestTarget();
}

/******************************************************************/
/*                                Get X                           */
/******************************************************************/

//From latestResult
double PhotonCamera::getBestTargetX() const
{
    return getBestTarget().GetYaw();
}

//From defined result

double PhotonCamera::getBestTargetX(photonlib::PhotonPipelineResult result) const
{
    return getBestTarget().GetYaw();
}

//From defined target

double PhotonCamera::getTargetX(photonlib::PhotonTrackedTarget target) const
{
    return target.GetYaw();
}

/******************************************************************/
/*                                Get Y                           */
/******************************************************************/

//From latestResult
double PhotonCamera::getBestTargetY() const
{
    return getBestTarget().GetPitch();
}

//From defined result

double PhotonCamera::getBestTargetY(photonlib::PhotonPipelineResult result) const
{
    return getBestTarget().GetPitch();
}

//From defined target

double PhotonCamera::getTargetY(photonlib::PhotonTrackedTarget target) const
{
    return target.GetPitch();
}

/******************************************************************/
/*                             Get Area                           */
/******************************************************************/

//From latestResult
double PhotonCamera::getBestTargetArea() const
{
    return getBestTarget().GetArea();
}

//From defined result

double PhotonCamera::getBestTargetArea(photonlib::PhotonPipelineResult result) const
{
    return getBestTarget().GetArea();
}

//From defined target

double PhotonCamera::getTargetArea(photonlib::PhotonTrackedTarget target) const
{
    return target.GetArea();
}

/******************************************************************/
/*                             Get Skew                           */
/******************************************************************/

//From latestResult
double PhotonCamera::getBestTargetSkew() const
{
    return getBestTarget().GetSkew();
}

//From defined result

double PhotonCamera::getBestTargetSkew(photonlib::PhotonPipelineResult result) const
{
    return getBestTarget().GetSkew();
}

//From defined target

double PhotonCamera::getTargetSkew(photonlib::PhotonTrackedTarget target) const
{
    return target.GetSkew();
}

/******************************************************************/
/*                             Get Pose                           */
/******************************************************************/

//From latestResult
auto PhotonCamera::getBestTargetPose() const
{
    return getBestTarget().GetCameraRelativePose();
}

//From defined result

auto PhotonCamera::getBestTargetPose(photonlib::PhotonPipelineResult result) const
{
    return getBestTarget().GetCameraRelativePose();
}

//From defined target

auto PhotonCamera::getTargetPose(photonlib::PhotonTrackedTarget target) const
{
    return target.GetCameraRelativePose();
}

/******************************************************************/
/*                         Extra Methods                          */
/******************************************************************/

bool PhotonCamera::hasTarget() const
{
    return camera.HasTargets();
}

void PhotonCamera::setLEDMode(LED_Mode mode)
{
    switch((int) mode)
    {
    case static_cast<int> (photonlib::LEDMode::kDefault): camera.SetLEDMode(photonlib::LEDMode::kDefault);
    case (int) photonlib::LEDMode::kBlink: camera.SetLEDMode(photonlib::LEDMode::kBlink);
    case (int) photonlib::LEDMode::kOff: camera.SetLEDMode(photonlib::LEDMode::kOff);
    case (int) photonlib::LEDMode::kOn: camera.SetLEDMode(photonlib::LEDMode::kOn);
    }
}

PhotonCamera::LED_Mode PhotonCamera::getLEDMode() const
{
    switch((int) camera.GetLEDMode())
    {
    case (int) LED_Mode::Default: return LED_Mode::Default;
    case (int) LED_Mode::Blink: return LED_Mode::Blink;
    case (int) LED_Mode::Force_Off: return LED_Mode::Force_Off;
    case (int) LED_Mode::Force_On: return LED_Mode::Force_On;
    }
}