/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <AngleConversion.h>
#include <utility/DriveDistanceTracker.h>

void Robot::RobotInit() 
{
    m_drivetrain.InitalShowToSmartDashboard();
    m_intake.OnRobotInit();
    m_shooter.OnRobotInit();
    // m_colorWheel.OnRobotInit();
    m_vision.OnRopotInit();

    m_driverSpeedConditioning.SetDeadband(kDefaultDeadband);
    m_driverSpeedConditioning.SetRange(kDefaultOutputOffset, 1.0);
    m_driverSpeedConditioning.SetExponent(kDefaultExponent);

    m_driverRotationConditioning.SetDeadband(kDefaultDeadband);
    m_driverRotationConditioning.SetRange(kDefaultOutputOffset, 1.0);
    m_driverRotationConditioning.SetExponent(kDefaultExponent);

    frc::SmartDashboard::PutNumber("Speed Deadband", kDefaultDeadband);
    frc::SmartDashboard::PutNumber("Speed Output Offset", kDefaultOutputOffset);
    frc::SmartDashboard::PutNumber("Speed Exponent", kDefaultExponent);

    frc::SmartDashboard::PutNumber("Rotation Deadband", kDefaultDeadband);
    frc::SmartDashboard::PutNumber("Rotation Output Offset", kDefaultOutputOffset);
    frc::SmartDashboard::PutNumber("Rotation Exponent", kDefaultExponent);

    frc::SmartDashboard::PutBoolean("Update Conditioning", false);

    frc::SmartDashboard::PutString("Robot State", "Traveling");

    m_stateManager = std::make_unique<StateManager>(m_intake, m_shooter, m_vision, m_drivetrain, m_driverJoystick, m_coPilot, 3);
}

void Robot::RobotPeriodic() 
{
    if (m_skips % 47)
    {
        // update PID values from the SmartDashboard
        m_drivetrain.UpdateFromSmartDashboard();
        if (frc::SmartDashboard::GetBoolean("Update Conditioning", false))
        {
            double ldb = frc::SmartDashboard::GetNumber("Speed Deadband", kDefaultDeadband);
            double loo = frc::SmartDashboard::GetNumber("Speed Output Offset", kDefaultOutputOffset);
            double lex = frc::SmartDashboard::GetNumber("Speed Exponent", kDefaultExponent);

            double rdb = frc::SmartDashboard::GetNumber("Rotation Deadband", kDefaultDeadband);
            double roo = frc::SmartDashboard::GetNumber("Rotation Output Offset", kDefaultOutputOffset);
            double rex = frc::SmartDashboard::GetNumber("Rotation Exponent", kDefaultExponent);

            m_driverSpeedConditioning.SetDeadband(ldb);
            m_driverSpeedConditioning.SetRange(loo, 1.0);
            m_driverSpeedConditioning.SetExponent(lex);

            m_driverRotationConditioning.SetDeadband(rdb);
            m_driverRotationConditioning.SetRange(roo, 1.0);
            m_driverRotationConditioning.SetExponent(rex);
        }
    }
    if (true /*m_skips % 51*/)
    {
        // update PID values from the SmartDashboard
        m_shooter.OnRobotPeriodic();
    }
    if (m_skips % 53)
    {
        // update PID values from the SmartDashboard
        m_intake.OnRobotPeriodic();
    }

    // m_colorWheel.UpdateColorSensorValues(m_skips);

    if (m_skips % 33)
    {
        m_vision.OnRobotPeriodic();
    }

    // Increment the m_skips variable for counting
    m_skips++;
}

void Robot::AutonomousInit()
{
    if(m_stateManager.get() != nullptr)
        m_autonomous = std::make_unique<Autonomous>(StartingPosition::Center, *m_stateManager, m_shooter, m_drivetrain);
}

void Robot::AutonomousPeriodic()
{
    if(m_autonomous.get() != nullptr)
        m_autonomous->Periodic();
}

void Robot::TeleopInit()
{
    m_autonomous.reset();
    m_stateManager->Reset();
    m_intake.SetIntakeSpeed(0);
    m_intake.SetConveyorSpeed(0);
    m_intake.SetKickerSpeed(0);
}

void Robot::TeleopPeriodic() 
{
    if (m_coPilot.GetRawButton(kTurnToTarget))
    {
        if (m_autoTurnTargetAngle == 0)
        {
            // read from camera
            m_autoTurnTargetAngle = m_vision.GetTargetXAngleDegrees() + m_drivetrain.GetAngle();
        }
        if (m_drivetrain.TryTurnToTargetAngle(m_autoTurnTargetAngle) == false)
        {
            m_autoTurnTargetAngle = 0;
        }
    }
    else
    {
    m_drivetrain.CurvatureDrive(
        m_driverSpeedConditioning.Condition(-m_driverJoystick.GetY()),
        m_driverRotationConditioning.Condition(m_driverJoystick.GetTwist()),
        m_quickTurn.Get());
    }
    
    m_shooter.SetSpeedFromThrottle(m_coPilot.GetThrottle());

    if (m_coPilot.GetRawButtonPressed(kSetAngle))
    {
        m_shooter.SetAngle(!m_shooter.GetAngle());
    }

    // When Firing Done
    if (m_coPilot.GetTriggerReleased())
    {
        m_stateManager->StartState(States::Traveling);
    }
    else if (m_shooter.CloseToSpeed() && m_coPilot.GetTriggerPressed())
    {
        m_stateManager->StartState(States::Firing);
    }

    if (m_coPilot.GetRawButtonPressed(kIntakeToggle))
    {
        if (m_intake.IsIntakeRunning())
        {
            m_stateManager->StartState(States::Traveling);
        }
        else
        {
            m_stateManager->StartState(States::Loading);
        }
    }

    // if (m_coPilot.GetRawButtonPressed(kEngageColorWheel))
    // {
    //     if (m_colorWheel.IsColorWheelEngaged())
    //     {
    //         SwitchState(Robot::States::Traveling);
    //     }
    //     else
    //     {
    //         SwitchState(Robot::States::ColorWheel);
    //     }
    // }

    // if (m_coPilot.GetRawButtonPressed(kClimbExtend))
    // {
    //     SwitchState(Robot::States::Climbing);
    // }

    m_stateManager->Periodic();
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
