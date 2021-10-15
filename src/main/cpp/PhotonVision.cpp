#include "PhotonVision.hpp"
#include "PhotonLib/PhotonCamera.hpp"

#include <iostream>

// Step 1
double PhotonCamera::getX() const { return table->GetNumber("targetYaw", 0.0); }
double PhotonCamera::getY() const { return table->GetNumber("targetPitch", 0.0); }
double PhotonCamera::getArea() const { return table->GetNumber("targetArea", 0.0); }
double PhotonCamera::getSkew() const { return table->GetNumber("targetSkew", 0.0); }
bool   PhotonCamera::hasTarget() const { return table->GetNumber("hasTarget", 0.0); }
void PhotonCamera::setLEDMode(PhotonCamera::LED_Mode mode) {
    // table->PutNumber("ledMode", static_cast<int>(mode));
     }
// Create camera
photonlib::PhotonCamera camera {"gloworm1771"};
// Step 3 - currently not working
/*
camera.SetLEDMode(photonlib::kOn);              // turn LEDs on
camera.TakeInputSnapshot();                     // take initial snapshot
camera.SetPipelineIndex(2);
*/