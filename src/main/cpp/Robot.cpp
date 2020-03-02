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

    UpdateActivePowerCells();
}

void Robot::UpdateActivePowerCells()
{
    frc::SmartDashboard::PutBoolean("Power Cell 1", m_powercellsCounted >= 1);
    frc::SmartDashboard::PutBoolean("Power Cell 2", m_powercellsCounted >= 2);
    frc::SmartDashboard::PutBoolean("Power Cell 3", m_powercellsCounted >= 3);
    frc::SmartDashboard::PutBoolean("Power Cell 4", m_powercellsCounted >= 4);
    frc::SmartDashboard::PutBoolean("Power Cell 5", m_powercellsCounted >= 5);
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
    ClearPressedAndReleasedOperatorButtons();
    m_powercellsCounted = 3;
    m_shooter.SetSpeed(2500);
}

void Robot::AutonomousPeriodic()
{
    if(m_shooter.CloseToSpeed() && m_currentState != States::Firing && 
       m_currentState != States::Traveling)
    {
        SwitchState(States::Firing);
    }
    else if(m_currentState == States::Firing && m_powercellsCounted == 0)
    {
        SwitchState(States::Traveling);
        m_shooter.SetSpeed(1000);
        m_skips = 0;
        m_drivetrain.SetSpeeds(-Drivetrain::kMaxVelocity * 0.5, 
                               -Drivetrain::kMaxVelocity * 0.5);
    }
    else if(m_currentState == States::Traveling && m_skips == 5)
    {
        m_drivetrain.SetSpeeds(0,0);
        m_autoTurnTargetAngle = m_drivetrain.GetAngle() + 90;
    }
    else if(m_currentState == States::Traveling && m_skips > 5 && m_autoTurnTargetAngle != 0)
    {
        if(m_drivetrain.TryTurnToTargetAngle(m_autoTurnTargetAngle) == false)
        {
            m_autoTurnTargetAngle = 0;
        }
    }
    DoCurrentState();
    m_skips++;
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
    if (m_coPilot.GetRawButton(kTurnToTarget))
    {
        if (m_autoTurnTargetAngle == 0)
        {
            // read from camera
            m_autoTurnTargetAngle = GetTargetXAngleDegrees() + m_drivetrain.GetAngle();
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
        SwitchState(Robot::States::Traveling);
    }
    else if (m_shooter.CloseToSpeed() && m_coPilot.GetTriggerPressed())
    {
        SwitchState(Robot::States::Firing);
    }

    if (m_coPilot.GetRawButtonPressed(kIntakeToggle))
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

    ProcessUnjammingButtonPresses();
   
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
    // m_colorWheel.EngageColorWheel(false);
    if (m_powercellsCounted <= 1)
    {
        m_shooter.SetAngle(false);
    }

    // needs shooter to idle speed

    frc::SmartDashboard::PutString("Robot State", "Traveling");
}

void Robot::TravelingPeriodic() 
{
    // Driving is not included in this because that will be just inside TeleopPeriodic
}

void Robot::FiringInit() 
{
    m_intake.ProcessStickySwitches();
    m_intake.GetSensorReleased(Intake::SensorLocation::Kicker);
    m_intake.SetIntakeSpeed(0);
    m_intake.SetConveyorSpeed(m_intake.GetConveyorFullSpeed());
    m_intake.SetKickerSpeed(m_intake.GetKickerFullSpeed());

    frc::SmartDashboard::PutString("Robot State", "Firing");
}

void Robot::FiringPeriodic() 
{
    m_intake.ProcessStickySwitches();
    if (m_intake.GetSensorReleased(Intake::SensorLocation::Kicker))
    {
        m_powercellsCounted--;
        UpdateActivePowerCells();
    }
}

void Robot::ClimbingInit() 
{
    TravelingInit();

    frc::SmartDashboard::PutString("Robot State", "Climbing");
}

void Robot::ClimbingPeriodic()
{
    if (m_coPilot.GetRawButton(kClimbExtend))
    {
        // extend climb mechanism
    }
    else if (m_coPilot.GetRawButton(kClimbRetract))
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
    // m_coldorWheel.EngageColorWheel(true);

    frc::SmartDashboard::PutString("Robot State", "Color Wheel");
}

void Robot::ColorWheelPeriodic()
{
    // if (m_colorWheel.IsSpinning())
    // {
    // }
    // else if (m_coPilot.GetRawButtonPressed(kSpinColorWheel))
    // {
    //     m_colorWheel.Spin(true);
    // }
    // else if (m_coPilot.GetRawButtonPressed(kGoToColor))
    // {
    //     m_colorWheel.SpinToColor();
    // }
    // else
    // {
    //     SwitchState(Robot::States::Traveling);
    // }
}

void Robot::LoadingInit()
{
    m_intake.ProcessStickySwitches();
    m_intake.GetSensorPressed(Intake::SensorLocation::NewPowercell);
    m_intake.GetSensorPressed(Intake::SensorLocation::SecuredPowercell);
    m_shooter.SetAngle(false);
    m_intake.SetConveyorSpeed(0);
    m_intake.SetKickerSpeed(m_intake.GetKickerFullSpeed());
    m_intake.SetIntakeSpeed(m_intake.GetIntakeFullSpeed());
    m_powercellsCounted = 0;

    frc::SmartDashboard::PutString("Robot State", "Loading");
}

void Robot::LoadingPeriodic()
{
    m_intake.ProcessStickySwitches();

    if(m_intake.GetSensorPressed(Intake::SensorLocation::NewPowercell))
    {
        if(m_powercellsCounted == 4)
        {
            m_powercellsCounted++;
            SwitchState(Robot::States::Traveling);
            UpdateActivePowerCells();
            return;
        }
        else
        {
            m_intake.SetIntakeSpeed(m_intake.GetIntakeFullSpeed() * 0.5);
            m_intake.SetConveyorSpeed(m_intake.GetConveyorFullSpeedWhenLoading());
        }
    }

    if (!m_intake.GetSensor(Intake::SensorLocation::Kicker))
    {
        m_intake.SetKickerSpeed(0);
    }

    if(m_intake.GetSensorPressed(Intake::SensorLocation::SecuredPowercell))
    {
        m_intake.SetConveyorSpeed(0);
        m_intake.SetIntakeSpeed(m_intake.GetIntakeFullSpeed());
        m_powercellsCounted++;

        if (m_powercellsCounted == 5)
        {
            SwitchState(Robot::States::Traveling);
        }
    }

    UpdateActivePowerCells();
}

void Robot::ProcessUnjammingButtonPresses()
{
    if (m_coPilot.GetRawButtonReleased(kReverseKicker))
    {
        SwitchState(Robot::States::Traveling);
    }
    else if (m_coPilot.GetRawButtonPressed(kReverseKicker))
    {
        SwitchState(Robot::States::Traveling);
        m_intake.SetIntakeSpeed(-m_intake.GetIntakeFullSpeed());
        m_intake.SetConveyorSpeed(-m_intake.GetConveyorFullSpeedWhenLoading());
        m_intake.SetKickerSpeed(-m_intake.GetKickerFullSpeed());
    }

    if (m_coPilot.GetRawButtonReleased(kReverseConveyor))
    {
        SwitchState(Robot::States::Traveling);
    }
    else if (m_coPilot.GetRawButtonPressed(kReverseConveyor))
    {
        SwitchState(Robot::States::Traveling);
        m_intake.SetIntakeSpeed(-m_intake.GetIntakeFullSpeed());
        m_intake.SetConveyorSpeed(-m_intake.GetConveyorFullSpeedWhenLoading());;
    }

    if (m_coPilot.GetRawButtonReleased(kReverseIntake))
    {
        SwitchState(Robot::States::Traveling);
    }
    else if (m_coPilot.GetRawButtonPressed(kReverseIntake))
    {
        SwitchState(Robot::States::Traveling);
        m_intake.SetIntakeSpeed(-m_intake.GetIntakeFullSpeed());
    }
}

void Robot::ClearPressedAndReleasedOperatorButtons()
{
    m_coPilot.GetTriggerReleased();
    m_coPilot.GetTriggerPressed();
    m_coPilot.GetRawButtonPressed(kIntakeToggle);
    m_coPilot.GetRawButtonPressed(kGoToColor);
    m_coPilot.GetRawButtonPressed(kEngageColorWheel);
    m_coPilot.GetRawButtonPressed(kSpinColorWheel);
    m_coPilot.GetRawButtonPressed(kClimbExtend);
    m_coPilot.GetRawButtonPressed(kSetAngle);
    m_coPilot.GetRawButtonPressed(kReverseConveyor);
    m_coPilot.GetRawButtonReleased(kReverseConveyor);
    m_coPilot.GetRawButtonPressed(kReverseIntake);
    m_coPilot.GetRawButtonReleased(kReverseIntake);
    m_coPilot.GetRawButtonPressed(kReverseKicker);
    m_coPilot.GetRawButtonReleased(kReverseKicker);
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
