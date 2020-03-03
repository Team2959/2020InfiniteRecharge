#pragma once

#include <utility/StateManager.h>
#include <subsystems/Shooter.h>
#include <subsystems/Drivetrain.h>
#include <utility/DriveDistanceTracker.h>
#include <frc/smartdashboard/SendableChooser.h>

enum StartingPosition
{
    Left,
    Right,
    Center
};

class Autonomous
{
private:
    StateManager& m_stateManager;
    Shooter& m_shooter;
    Drivetrain& m_driveTrain;
    StartingPosition m_startingPosition = Center;
    double m_autoTurnTargetAngle = 0.0;
    DriveDistanceTracker m_autoDriveDistanceTracker {};

    // SmartDashboard auton mode selector
    frc::SendableChooser<std::string> m_chooser;
    const std::string kAutoLeft = "Left";
    const std::string kAutoCenter = "Center";
    const std::string kAutoRight = "Right";

public:
    Autonomous(StateManager& stateManager, Shooter& shooter, Drivetrain& driveTrain);

    void OnRobotInit();
    void OnAutoInit();
    void Periodic();
};
