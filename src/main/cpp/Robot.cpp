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
    m_colorWheel.OnRobotInit();

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

    m_colorWheel.UpdateColorSensorValues(m_skips);

    // Increment the m_skips variable for counting
    m_skips++;
}

void Robot::AutonomousInit()
{
    ClearPressedAndReleasedOperatorButtons();
}

void Robot::AutonomousPeriodic()
{
    m_intake.ProcessStickySwitches();
}

void Robot::TeleopInit()
{
    // Remove for competition bot??, right now getting around trigger pressed and disabled mode
    ClearPressedAndReleasedOperatorButtons();
    m_intake.SetIntakeSpeed(0);
    m_intake.SetConveyorSpeed(0);
    m_intake.SetKickerSpeed(0);
}

void Robot::TeleopPeriodic() 
{
    m_intake.ProcessStickySwitches();

    m_drivetrain.SetSpeeds(m_conditioningDriverJoysticks.Condition(m_leftDriverJoystick.GetY()) * Drivetrain::kMaxVelocity,
                           m_conditioningDriverJoysticks.Condition(m_rightDriverJoystick.GetY()) * Drivetrain::kMaxVelocity);

    // using the throttle for now
    auto targetSpeed = (m_leftDriverJoystick.GetThrottle() + 1) * Shooter::kHalfMaxVelocity;
    frc::SmartDashboard::PutNumber("Throttle Target Speed", targetSpeed);
    m_shooter.SetSpeed(targetSpeed);

    if (m_coPilot.GetRawButtonPressed(6))
    {
        m_shooter.SetAngle(!m_shooter.GetAngle());
    }

    // When Firing Done
    if (m_rightDriverJoystick.GetTriggerReleased())
    {
        SwitchState(Robot::States::Loading);
    }
    else if (m_shooter.CloseToSpeed() && m_rightDriverJoystick.GetTriggerPressed())
    {
        SwitchState(Robot::States::Firing);
    }

    if (m_rightDriverJoystick.GetRawButtonPressed(2))
    {
        if (m_intake.IsIntakeRunning())
        {
            SwitchState(Robot::States::Traveling);
        }
        else
        {
            SwitchState(Robot::States::Loading);
        }
    }

    if (m_coPilot.GetRawButtonPressed(2))
    {
        if (m_colorWheel.IsColorWheelEngaged())
        {
            SwitchState(Robot::States::Traveling);
        }
        else
        {
            ColorWheelInit();
        }
    }

    if (m_coPilot.GetRawButtonPressed(5))
    {
        SwitchState(Robot::States::Climbing);
    }

    if (m_leftDriverJoystick.GetRawButtonReleased(9))
    {
        SwitchState(Robot::States::Traveling);
    }
    else if (m_leftDriverJoystick.GetRawButtonPressed(9))
    {
        SwitchState(Robot::States::Traveling);
        m_intake.SetIntakeSpeed(-m_intake.GetIntakeFullSpeed());
        m_intake.SetConveyorSpeed(-m_intake.GetConveyorFullSpeedWhenLoading());;
    }

    if (m_leftDriverJoystick.GetRawButtonReleased(11))
    {
        SwitchState(Robot::States::Traveling);
    }
    else if (m_leftDriverJoystick.GetRawButtonPressed(11))
    {
        SwitchState(Robot::States::Traveling);
        m_intake.SetIntakeSpeed(-m_intake.GetIntakeFullSpeed());
    }

    DoCurrentState();
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
    m_colorWheel.EngageColorWheel(false);
    m_shooter.SetAngle(false);

    // needs shooter to idle speed
}

// Driving is not included in this because that will be just inside TeleopPeriodic
void Robot::TravelingPeriodic() 
{
}

void Robot::FiringInit() 
{
    m_kickerPulseCounts = 1;
    m_intake.SetIntakeSpeed(0);
    m_intake.SetConveyorSpeed(m_intake.GetConveyorFullSpeed());
    m_intake.SetKickerSpeed(m_intake.GetKickerFullSpeed());
}

void Robot::FiringPeriodic() 
{
    if (m_kickerPulseCounts > 0)
    {
        auto currentCount = m_kickerPulseCounts;
        m_kickerPulseCounts++;
        if (currentCount > m_intake.GetKickerPulseCycles())
        {
            if (currentCount <= m_intake.GetKickerPulseCycles() + m_intake.GetKickerPauseCycles())
            {
                m_intake.SetKickerSpeed(0);
            }
            else
            {
                m_intake.SetKickerSpeed(m_intake.GetKickerFullSpeed());
                m_kickerPulseCounts = 0;
            }
        }
    }
}

void Robot::ClimbingInit() 
{
    TravelingInit();
}

void Robot::ClimbingPeriodic()
{
    if (m_coPilot.GetRawButton(5))
    {
        // extend climb mechanism
    }
    else if (m_coPilot.GetRawButton(7))
    {
        // raise bot from floor
    }
    else
    {
        // motor off
    }
    
}

void Robot::ColorWheelInit()
{
    TravelingInit();
    m_colorWheel.EngageColorWheel(true);
}

void Robot::ColorWheelPeriodic()
{
    if (m_colorWheel.IsSpinning())
    {
    }
    else if (m_coPilot.GetRawButtonPressed(3))
    {
        m_colorWheel.Spin(true);
    }
    else if (m_coPilot.GetRawButtonPressed(1))
    {
        m_colorWheel.SpinToColor();
    }
    else
    {
        SwitchState(Robot::States::Traveling);
    }
}

void Robot::LoadingInit()
{
    m_shooter.SetAngle(false);
    m_intake.SetConveyorSpeed(0);
    m_intake.SetKickerSpeed(0);
    m_intake.SetIntakeSpeed(m_intake.GetIntakeFullSpeed());
    m_powercellsCounted = 0;
}

void Robot::LoadingPeriodic()
{
    if(m_intake.GetSensorPressed(Intake::SensorLocation::NewPowercell))
    {
        m_intake.SetIntakeSpeed(m_intake.GetIntakeFullSpeed() * 0.5);
        m_intake.SetConveyorSpeed(m_intake.GetConveyorFullSpeedWhenLoading());

        if (m_intake.GetSensor(Intake::SensorLocation::Kicker))
        {
            m_intake.SetKickerSpeed(m_intake.GetKickerFullSpeed() * 0.5);
        }
        else
        {
            m_intake.SetKickerSpeed(0);
        }
    }

    if (!m_intake.GetSensor(Intake::SensorLocation::Kicker))
    {
        m_intake.SetKickerSpeed(0);
    }

    if(m_intake.GetSensorPressed(Intake::SensorLocation::SecuredPowercell))
    {
        m_intake.SetKickerSpeed(0);
        m_intake.SetConveyorSpeed(0);
        m_intake.SetIntakeSpeed(m_intake.GetIntakeFullSpeed());
        m_powercellsCounted++;

        if (m_powercellsCounted == 5)
        {
            SwitchState(Robot::States::Traveling);
            return;
        }
    }
}

void Robot::ClearPressedAndReleasedOperatorButtons()
{
    m_rightDriverJoystick.GetTriggerReleased();
    m_rightDriverJoystick.GetTriggerPressed();
    m_rightDriverJoystick.GetRawButtonPressed(2);
    m_leftDriverJoystick.GetRawButtonPressed(9);
    m_leftDriverJoystick.GetRawButtonReleased(9);
    m_leftDriverJoystick.GetRawButtonPressed(11);
    m_leftDriverJoystick.GetRawButtonReleased(11);
    m_coPilot.GetRawButtonPressed(1);
    m_coPilot.GetRawButtonPressed(2);
    m_coPilot.GetRawButtonPressed(3);
    m_coPilot.GetRawButtonPressed(5);
    m_coPilot.GetRawButtonPressed(6);
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
