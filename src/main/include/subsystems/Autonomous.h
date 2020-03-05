#pragma once

#include <utility/StateManager.h>
#include <subsystems/Shooter.h>
#include <subsystems/Drivetrain.h>
#include <utility/DriveDistanceTracker.h>
#include <frc/smartdashboard/SendableChooser.h>

enum AutoProgram
{
    FireAndForward,
    FireAndBackward,
    RightWithTrench,
    CenterWithTrench,
    LeftWithTrench,
    WallAndFire
};

class Autonomous
{
private:
    StateManager& m_stateManager;
    Shooter& m_shooter;
    Drivetrain& m_driveTrain;

    AutoProgram m_selectedProgram = FireAndForward;
    int m_step = 0;
    double m_autoTurnTargetAngle = 0.0;
    DriveDistanceTracker m_autoDriveDistanceTracker {};

    // SmartDashboard auton mode selector
    frc::SendableChooser<std::string> m_chooser;
    const std::string kFireAndForward = "Fire and Forward";
    const std::string kFireAndBackward = "Fire and Backward";
    const std::string kRightWithTrench = "Right with Trench";
    const std::string kCenterWithTrench = "Center with Trench";
    const std::string kLeftWithTrench = "Left with Trench";
    const std::string kWallAndFire = "Wall and Fire";

    void FireAndForwardPeriodic();
    void FireAndBackwardPeriodic();
    void RightWithTrenchPeriodic();
    void CenterWithTrenchPeriodic();
    void LeftWithTrenchPeriodic();
    void WallAndFirePeriodic();

public:
    Autonomous(StateManager& stateManager, Shooter& shooter, Drivetrain& driveTrain);

    void OnRobotInit();
    void OnAutoInit();
    void Periodic();
};
