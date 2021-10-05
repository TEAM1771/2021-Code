#include "Constants.hpp"
#include <frc/Solenoid.h>
#include <rev/CANSparkMax.h>

namespace Intake
{
//public function declarations
    void init();
    void drive(INTAKE::DIRECTION mode);
    void deploy(bool val);

    [[nodiscard]] bool isIntakeDown();
}