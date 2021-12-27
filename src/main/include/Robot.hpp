/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//#include "Limelight.hpp"
#include <frc/TimedRobot.h>
#include <frc/livewindow/LiveWindow.h>
#include "Constants.hpp"
//#include "Average.hpp"

class Robot : public frc::TimedRobot
{
public:
    Robot();
    void AutonomousInit() override;
    void AutonomousPeriodic() override;

    void TeleopInit() override;
    void TeleopPeriodic() override;

    void DisabledInit() override;
    void DisabledPeriodic() override;

    void TestInit() override;
    void TestPeriodic() override;

    void ButtonManager();

    // returns true when aimed at goal
    bool aim(TURRET::POSITION);
    void ThreeBall();
    void FiveBall();
    void SixBall();
    void EightBall();
//    void TenBall();
    void ThirteenBall();

    void TestTrajectory();

    bool ShooterTempUpdate();

private:
    frc::LiveWindow& m_lw = *frc::LiveWindow::GetInstance();

};
