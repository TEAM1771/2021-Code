#include "Constants.hpp"
#include <frc/Solenoid.h>
#include <rev/CANSparkMax.h>

class Intake
{
    frc::Solenoid intakeair { INTAKE::PCM_PORT };
    bool          intakeDeployed = false;

    rev::CANSparkMax wheels { INTAKE::PORT, rev::CANSparkMaxLowLevel::MotorType::kBrushless };

public:
    Intake();
    void drive(INTAKE::DIRECTION mode);
    void deploy(bool val);

    [[nodiscard]] bool isIntakeDown() const;
};