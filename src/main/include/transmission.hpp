#pragma once

#include "Constants.hpp"
#include <ctre\Phoenix.h>
#include <rev\CANSparkMax.h>

class Transmission
{
    WPI_TalonFX falcon;

public:
    Transmission(int falcon_adr);
    double getEncoderDistance();
    void   setEncoderDistance(double distance);

    WPI_TalonFX* operator->(); // used to access the falcons directly

    void Set(double val);

    TalonFXSensorCollection& sensors;
};