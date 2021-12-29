#include "Climber.hpp"
#include "PID_CANSparkMax.hpp"
#include "Constants.hpp"


/******************************************************************/
/*                             Constants                          */
/******************************************************************/
inline static constexpr int PORT_1 = 9;
inline static constexpr int PORT_2 = 35;

inline static constexpr auto IDLE_MODE = rev::CANSparkMax::IdleMode::kBrake;

inline static constexpr double P = 0.1771;
inline static constexpr double I = 0.0;
inline static constexpr double D = 0.0;

inline static constexpr double MAX_OUTPUT = 1;

//Climber Position Constants are held in Climber.hpp

/******************************************************************/
/*                          Non-constant Vars                     */
/******************************************************************/
inline static PID_CANSparkMax climber_1 { PORT_1, rev::CANSparkMaxLowLevel::MotorType::kBrushless };
inline static PID_CANSparkMax climber_2 { PORT_2, rev::CANSparkMaxLowLevel::MotorType::kBrushless };

/******************************************************************/
/*                      Non Static Functions                      */
/******************************************************************/
void Climber::init()
{
    climber_1.RestoreFactoryDefaults();
    climber_2.RestoreFactoryDefaults();

    climber_1.SetIdleMode(IDLE_MODE);
    climber_2.SetIdleMode(IDLE_MODE);

    climber_1.SetP(P);
    climber_1.SetI(I);
    climber_1.SetD(D);
    climber_1.SetOutputRange(-MAX_OUTPUT, MAX_OUTPUT);
    climber_1.SetPositionRange(Climber::POSITION::ZERO, Climber::POSITION::UP);
    climber_1.SetTarget(Climber::POSITION::ZERO);

    climber_2.SetP(P);
    climber_2.SetI(I);
    climber_2.SetD(D);
    climber_2.SetOutputRange(-MAX_OUTPUT, MAX_OUTPUT);
    climber_2.SetPositionRange(-Climber::POSITION::UP, Climber::POSITION::ZERO);
    climber_2.SetTarget(Climber::POSITION::ZERO);
}

void Climber::set(Climber::POSITION position)
{
    climber_1.SetTarget(position, rev::ControlType::kPosition);
    climber_2.SetTarget(-position, rev::ControlType::kPosition);
}

void Climber::joystickControl(double val)
{
    climber_1.Set(val);
    climber_2.Set(-val);
    printStatus();
}

void Climber::printStatus()
{
    std::cout << "Climber 1: " << climber_1.encoder.GetPosition() << std::endl;
    std::cout << "Climber 2: " << climber_2.encoder.GetPosition() << std::endl;
}

void Climber::buttonManager()
{
    static bool hasBeenPressed = false;
    if(BUTTON::CLIMBER::RAISE && BUTTON::oStick.GetThrottle() < 0)
    {
        hasBeenPressed = true;
        set(Climber::POSITION::UP);
    }
    else if(hasBeenPressed)
        set(Climber::POSITION::DOWN);
}