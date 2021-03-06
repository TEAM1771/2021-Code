/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Climber.hpp"
#include "Drivetrain.hpp"
#include "Hood.hpp"
#include "Hopper.hpp"
#include "Intake.hpp"
#include "Limelight.hpp"
#include "ShooterWheel.hpp"
#include "Turret.hpp"
#include <frc/Joystick.h>
#include <frc/PWMVictorSPX.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/smartdashboard/smartdashboard.h>
#include <memory>

class Robot : public frc::TimedRobot
{
public:
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;

    void DisabledInit() override;
    void DisabledPeriodic() override;

    void TestPeriodic() override;

    void ButtonManager();

    // returns true when aimed at goal
    bool aim(TURRET::POSITION);
    void ThreeBall();
    void FiveBall();

private:
    Drivetrain   drivetrain;
    LimeLight    limelight;
    Hood         hood { limelight };
    Turret       turret { limelight };
    Hopper       hopper;
    Climber      climber;
    Intake       intake;
    ShooterWheel shooter_wheel;

    frc::LiveWindow& m_lw = *frc::LiveWindow::GetInstance();
};
