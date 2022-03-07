#include <functional>

// allows functions outside of robot to know if the robot is enabled and more
namespace RobotState
{
    inline std::function<bool()> isEnabled;
    inline std::function<bool()> isDisabled;
    inline std::function<bool()> isAutonomous;
    inline std::function<bool()> isAutonomousEnabled;
    inline std::function<bool()> isOperatorControl;
    inline std::function<bool()> isOperatorControlEnabled;
    inline std::function<bool()> isTest;
}