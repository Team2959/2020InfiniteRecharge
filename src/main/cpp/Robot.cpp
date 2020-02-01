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

    m_conditioning.SetDeadband(0.05);
    m_conditioning.SetExponent(5);
}

void Robot::RobotPeriodic() 
{
    if(m_skips % 50)
    {
        // update PID values from the SmartDashboard
        m_drivetrain.UpdateFromSmartDashboard();
    }

// Increment the m_skips variable for counting
    m_skips++;
}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() 
{
    m_drivetrain.TankDrive(m_conditioning.Condition(m_left.GetY()),
                           m_conditioning.Condition(m_right.GetY()));
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
