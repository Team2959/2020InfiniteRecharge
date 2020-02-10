/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() 
{
    m_drivetrain.InitalShowToSmartDashboard();
    m_intake.OnRobotInit();
    m_shooter.OnRobotInit();

    m_conditioningDriverJoysticks.SetDeadband(0.05);
    m_conditioningDriverJoysticks.SetExponent(5);
}

void Robot::RobotPeriodic() 
{
    if(m_skips % 50)
    {
        // update PID values from the SmartDashboard
        m_drivetrain.UpdateFromSmartDashboard();
    }
    if(m_skips % 51)
    {
        // update PID values from the SmartDashboard
        m_shooter.OnRobotPeriodic();
    }
    if(m_skips % 53)
    {
        // update PID values from the SmartDashboard
        m_intake.OnRobotPeriodic();
    }

    // Increment the m_skips variable for counting
    m_skips++;
}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
    // Remove for competition bot??, right now getting around trigger pressed and disabled mode
    m_rightDriverJoystick.GetTriggerReleased();
    m_rightDriverJoystick.GetTriggerPressed();
    m_intake.SetIntakeSpeed(0);
    m_intake.SetConveyorSpeed(0);
    m_intake.SetKickerSpeed(0);
}

void Robot::TeleopPeriodic() 
{
    m_drivetrain.SetSpeeds(m_conditioningDriverJoysticks.Condition(m_leftDriverJoystick.GetY()) * Drivetrain::kMaxVelocity,
                           m_conditioningDriverJoysticks.Condition(m_rightDriverJoystick.GetY()) * Drivetrain::kMaxVelocity);

    // using the throttle for now
    auto targetSpeed = (m_rightDriverJoystick.GetThrottle() + 1) * Shooter::kHalfMaxVelocity;
    frc::SmartDashboard::PutNumber("Throttle Target Speed", targetSpeed);
    m_shooter.SetSpeed(targetSpeed);

    if(m_leftDriverJoystick.GetRawButtonPressed(1))
    {
        m_shooter.SetAngle(!m_shooter.GetAngle());
    }

    // if(m_intake.GetSensor(Intake::SensorLocation::End))
    // {
    //     m_intake.SetKickerSpeed(0);
    // }

    if(m_rightDriverJoystick.GetTriggerReleased())
    {
        m_intake.SetKickerSpeed(0);
        m_intake.SetConveyorSpeed(0);
    }
    else if(m_shooter.CloseToSpeed() && m_rightDriverJoystick.GetTriggerPressed())
    {
        m_intake.SetIntakeSpeed(0);
        m_intake.SetKickerSpeed(1);
        m_intake.SetConveyorSpeed(1);
    }

    if(m_skips % 51)
    {
        m_shooter.OnTeleOpPeriodicDebug();
    }
    if(m_skips % 53)
    {
        m_intake.OnTeleOpPeriodicDebug();
    }
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
