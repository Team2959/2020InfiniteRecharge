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
    if (m_skips % 50)
    {
        // update PID values from the SmartDashboard
        m_drivetrain.UpdateFromSmartDashboard();
    }
    if (m_skips % 51)
    {
        // update PID values from the SmartDashboard
        m_shooter.OnRobotPeriodic();
    }
    if (m_skips % 53)
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

    if (m_leftDriverJoystick.GetRawButtonPressed(1))
    {
        m_shooter.SetAngle(!m_shooter.GetAngle());
    }

    // When Firing Done
    if (m_rightDriverJoystick.GetTriggerReleased())
    {
        m_intake.SetIntakeSpeed(1);
        m_intake.SetKickerSpeed(0);
        m_intake.SetConveyorSpeed(0);
    }
    else if (m_shooter.CloseToSpeed() && m_rightDriverJoystick.GetTriggerPressed())
    {
        m_intake.SetIntakeSpeed(0);
        m_intake.SetConveyorSpeed(frc::SmartDashboard::GetNumber("Conveyor: Speed", 0.9));
        m_intake.SetKickerSpeed(frc::SmartDashboard::GetNumber("Conveyor: Kicker Speed", 0.9));
        // m_intake.SetKickerSpeed(1);
        // m_intake.SetConveyorSpeed(1);
    }

    if(m_skips % 51)
    {
        m_shooter.OnTeleOpPeriodicDebug();
    }
    if (m_rightDriverJoystick.GetRawButtonPressed(2))
    {
        if (m_intake.IsIntakeRunning())
        {
            m_intake.SetIntakeSpeed(0);
            m_intake.SetConveyorSpeed(0);
            m_intake.SetKickerSpeed(0);
        }
        else
        {
            m_intake.SetIntakeSpeed(m_intake.GetIntakeFullSpeed());
            m_intake.SetConveyorSpeed(m_intake.GetConveyorFullSpeed());
            m_intake.SetKickerSpeed(m_intake.GetKickerFullSpeed());
        }
    }
}

void Robot::TestPeriodic() {}

void Robot::SwitchState(Robot::States state)
{
    m_currentState = state;
    if(m_currentState == States::Firing) FiringInit();
    else if(m_currentState == States::ColorWheel) ColorWheelInit();
    else if(m_currentState == States::Traveling) TravelingInit();
    else if(m_currentState == States::Loading) LoadingInit();
    else if(m_currentState == States::Climbing) ClimbingInit();
}

void Robot::DoCurrentState()
{
    if(m_currentState == States::Firing) FiringPeriodic();
    else if(m_currentState == States::ColorWheel) ColorWheelPeriodic();
    else if(m_currentState == States::Traveling) TravelingPeriodic();
    else if(m_currentState == States::Loading) LoadingPeriodic();
    else if(m_currentState == States::Climbing) ClimbingPeriodic();
}

void Robot::TravelingInit()
{
    m_intake.SetIntakeSpeed(0);
    m_intake.SetKickerSpeed(0);
    m_intake.SetConveyorSpeed(0);

    // needs shooter still
}

// Driving is not included in this because that will be just inside TeleopPeriodic
void Robot::TravelingPeriodic() 
{
}

void Robot::FiringInit() 
{
}

void Robot::FiringPeriodic() 
{
}

void Robot::ClimbingInit() 
{
}

void Robot::ClimbingPeriodic()
{
}

void Robot::ColorWheelInit()
{
}

void Robot::ColorWheelPeriodic()
{
}

void Robot::LoadingInit()
{
    m_intake.SetConveyorSpeed(0);
    m_intake.SetKickerSpeed(0);
    m_intake.SetIntakeSpeed(1);
}

void Robot::LoadingPeriodic()
{
    if(m_intake.GetSensor(Intake::SensorLocation::NewPowercell) &&
        !m_intake.GetSensorPressed(Intake::SensorLocation::SecuredPowercell))
    {
        m_intake.SetConveyorSpeed(1);
    }
    else
    {
        m_intake.SetConveyorSpeed(0);
        m_powercellsCounted++;
    }

    if(m_intake.GetSensor(Intake::SensorLocation::NewPowercell) && 
        m_intake.GetSensor(Intake::SensorLocation::StartKicker) &&
        !m_intake.GetSensor(Intake::SensorLocation::StopKicker))
    {
        m_intake.SetKickerSpeed(1);
    }
    else
    {
        m_intake.SetKickerSpeed(0);
    }

    if(m_powercellsCounted < 5)
    {
        m_intake.SetIntakeSpeed(1);
    }
    else
    {
        m_intake.SetIntakeSpeed(0);
    }
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
