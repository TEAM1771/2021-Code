#include "PhotonVision.hpp"

#include <iostream>

double PhotonCamera::getX() const { return table->GetNumber("targetYaw", 0.0); }
double PhotonCamera::getY() const { return table->GetNumber("targetPitch", 0.0); }
double PhotonCamera::getArea() const { return table->GetNumber("targetArea", 0.0); }
double PhotonCamera::getSkew() const { return table->GetNumber("targetSkew", 0.0); }
bool   PhotonCamera::hasTarget() const { return table->GetNumber("hasTarget", 0.0); }
void PhotonCamera::setLEDMode(PhotonCamera::LED_Mode mode) {
    // table->PutNumber("ledMode", static_cast<int>(mode));
     }
