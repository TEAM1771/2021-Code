#include "transmission.hpp"

Transmission::Transmission(int falcon_adr)
    : falcon { falcon_adr }
    , sensors { falcon.GetSensorCollection() }
{
    falcon.SetNeutralMode(TRANSMISSION::IDLE_MODE);
}

WPI_TalonFX* Transmission::operator->()
{
    return &falcon;
}

void Transmission::Set(double val)
{
    falcon.Set(ControlMode::PercentOutput, val);
}

double Transmission::getEncoderDistance()
{
    return sensors.GetIntegratedSensorPosition() / 2048.0;
}

void Transmission::setEncoderDistance(double distance)
{
    sensors.SetIntegratedSensorPosition(distance * 2048);
}
