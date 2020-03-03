#pragma once

#include <utility/StateManager.h>
#include <subsystems/Shooter.h>
#include <subsystems/Drivetrain.h>
#include <utility/DriveDistanceTracker.h>

enum StartingPosition
{
    Left,
    Right,
    Center
};

class Autonomous
{
private:
    std::unique_ptr<DriveDistanceTracker> m_autoDriveDistanceTracker;
    StartingPosition m_startingPosition;
    StateManager& m_stateManager;
    Shooter& m_shooter;
    Drivetrain& m_driveTrain;
    double m_autoTurnTargetAngle = 0.0;
public:
    Autonomous(StartingPosition startingPostion, StateManager& stateManager, Shooter& shooter, Drivetrain& driveTrain);
    void Periodic();
};
