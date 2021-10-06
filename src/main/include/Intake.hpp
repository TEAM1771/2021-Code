#include "Constants.hpp"

namespace Intake
{
    void init();
    void drive(INTAKE::DIRECTION mode);
    void deploy(bool val);

    [[nodiscard]] bool isIntakeDown();
} // namespace Intake