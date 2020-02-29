/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>

// TO DO:  Specify correct values for CameraAngle, CameraHeight, and TargetHeight
constexpr double CameraAngle{ DegreesToRadians(30.0) }; // Angle in radians of the vertical elevation of the camera
constexpr double CameraHeight{ 19.5 };                  // Height in inches of the camera above the floor.
constexpr double TargetHeight{ 98.25 };                 // Height in inches of the target above the floor.

void Robot::RobotInit() 
{
    m_drivetrain.InitalShowToSmartDashboard();
    m_intake.OnRobotInit();
    m_shooter.OnRobotInit();
    // m_colorWheel.OnRobotInit();

    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("limelight");
    m_txEntry = table->GetEntry("tx");
    m_tyEntry = table->GetEntry("ty");
    frc::SmartDashboard::PutNumber("Auto Turn Multiplier", m_autoTurnMultiplier);
    frc::SmartDashboard::PutNumber("Auto Turn Angle Adjust", m_autoTurnDegrees);

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

    // Increment the m_skips variable for counting
    m_skips++;


    if (m_skips % 33)
    {
        // TO DO:  Remove this code once vision testing is complete
        auto    isTargetValid{ IsTargetValid() };
        frc::SmartDashboard::PutString("Vision TV Present", isTargetValid ? "Yes" : "No");
        if(isTargetValid)
        {
            frc::SmartDashboard::PutNumber("Vision TX Angle", RadiansToDegrees(GetTargetXAngle()));
            frc::SmartDashboard::PutNumber("Vision TY Angle", RadiansToDegrees(GetTargetYAngle()));
            frc::SmartDashboard::PutNumber("Vision Distance", GetTargetDistance());
        }
        else
        {
            frc::SmartDashboard::PutString("Vision TX Angle", "");
            frc::SmartDashboard::PutString("Vision TY Angle", "");
            frc::SmartDashboard::PutString("Vision Distance", "");
        }

        m_autoTurnMultiplier = frc::SmartDashboard::GetNumber("Auto Turn Multiplier", kDefaultAutoTurnMultiplier);
        m_autoTurnDegrees = frc::SmartDashboard::PutNumber("Auto Turn Angle Adjust", kDefaultAutoTurnDegrees);
    }
}

// TO DO:  Implement code that calls GetDistanceAngle, GetAngleDistance & GetMotorOutputForAimAndDrive and uses their results
// Adapted from http://docs.limelightvision.io/en/latest/cs_estimating_distance.html
double Robot::GetTargetAngleFromDistance(double distance)
{
    if(distance == 0.0) // Protect against zero distance and division by zero
        return 0.0;
    return RadiansToDegrees(std::atan((TargetHeight - CameraHeight) / distance)) - CameraAngle;  // Do some trigonometry to compute the angle that corresponds to the input distance
}

// Adapted from http://docs.limelightvision.io/en/latest/cs_estimating_distance.html
double Robot::GetTargetDistanceFromAngle(double angle)
{
    auto    tangent{ std::tan(DegreesToRadians(angle + CameraAngle)) };
    if(tangent == 0.0)  // Avoid any possible division by zero
        return 0.0;
    return (TargetHeight - CameraHeight) / tangent;  // Do some trigonometry to compute the distance that corresponds to the input angle
}

// TO DO:  Specify correct KpAim, KpDistance, & MinAimCommand values
// Adapted from https://docs.limelightvision.io/en/latest/cs_aimandrange.html
std::tuple<double, double> Robot::GetMotorOutputForAimAndDrive(double targetY)
{
    static const double KpAim{ -DegreesToRadians(0.1) };                // These are the coefficients for tuning the response to our target error
    static const double KpDistance{ -DegreesToRadians(0.1) };
    static const double MinAimCommand{ 0.05 };                          // The minimum amount of response if we are turning
    static const double LimitAngle{ DegreesToRadians(1.0) };            // If our angles are within this difference of zero, then we are on target
    auto                targetXAngle{ GetTargetXAngle() };
    auto                heading_error{ 0.0 - targetXAngle };            // Aim for tx == 0.0f
    auto                distance_error{ 0.0};//targetY - GetTargetYAngle() };  // Aim for ty == targetY
    auto                distance_adjust{ KpDistance * distance_error }; // Compute our distance adjustment, and
    double              steering_adjust;                                // Will hold our steering adjustment

    if (targetXAngle > LimitAngle)                                      // If the target is to the right, we need to turn to the left
        steering_adjust = KpAim * heading_error - MinAimCommand;
    else if (targetXAngle < LimitAngle)                                 // If the target is to the left, we need to turn to the right
        steering_adjust = KpAim * heading_error + MinAimCommand;
    else                                                                // Don't turn if +/- 1.0 degrees from crosshairs
        steering_adjust = 0.0f;
    return std::make_tuple(steering_adjust + distance_adjust,  steering_adjust + distance_adjust);  // Apply the distance adjustment to each component
}

void Robot::TurnToTarget()
{
    static const double LimitAngle{ DegreesToRadians(3.0) };            // If our angles are within this difference of zero, then we are on target
    auto tx = GetTargetXAngle();
    if (std::fabs(tx) > LimitAngle)
    {
        auto txDegrees = RadiansToDegrees(tx);
        auto turnSpeed = m_autoTurnMultiplier * txDegrees / m_autoTurnDegrees;
        m_drivetrain.CurvatureDrive(0.0, turnSpeed, true);
        frc::SmartDashboard::PutNumber("Turn To Target Angle", txDegrees);
        frc::SmartDashboard::PutNumber("Turn To Target Speed", turnSpeed);
    }
    else
    {
        m_drivetrain.CurvatureDrive(0, 0, false);
    }
}

double Robot::GetTargetXAngle() const
{
    if(!IsTargetValid())
        return std::nan("");
    return DegreesToRadians(m_txEntry.GetDouble(0.0)); 
}

double Robot::GetTargetYAngle() const 
{
    if(!IsTargetValid())
        return std::nan("");
    return DegreesToRadians(m_tyEntry.GetDouble(0.0)); 
}

void Robot::AutonomousInit()
{
    ClearPressedAndReleasedOperatorButtons();
}

void Robot::AutonomousPeriodic()
{
    DoCurrentState();
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
        TurnToTarget();
    }
    else
    {
    m_drivetrain.CurvatureDrive(
        m_driverSpeedConditioning.Condition(-m_driverJoystick.GetY()),
        m_driverRotationConditioning.Condition(m_driverJoystick.GetTwist()),
        m_quickTurn.Get());
    }
    
    m_shooter.SetSpeedFromThrottle(m_throttle.GetThrottle());

    if (m_coPilot.GetRawButtonPressed(kSetAngle))
    {
        m_shooter.SetAngle(!m_shooter.GetAngle());
    }

    // When Firing Done
    if (m_driverJoystick.GetTriggerReleased())
    {
        SwitchState(Robot::States::Traveling);
    }
    else if (m_shooter.CloseToSpeed() && m_driverJoystick.GetTriggerPressed())
    {
        SwitchState(Robot::States::Firing);
    }

    if (m_driverJoystick.GetRawButtonPressed(kIntakeToggle))
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
    m_shooter.SetAngle(false);

    // needs shooter to idle speed

    frc::SmartDashboard::PutString("Robot State", "Traveling");
}

void Robot::TravelingPeriodic() 
{
    // Driving is not included in this because that will be just inside TeleopPeriodic
}

void Robot::FiringInit() 
{
    m_intake.SetIntakeSpeed(0);
    m_intake.SetConveyorSpeed(m_intake.GetConveyorFullSpeed());
    m_intake.SetKickerSpeed(m_intake.GetKickerFullSpeed());

    frc::SmartDashboard::PutString("Robot State", "Firing");
}

void Robot::FiringPeriodic() 
{
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
    m_driverJoystick.GetTriggerReleased();
    m_driverJoystick.GetTriggerPressed();
    m_driverJoystick.GetRawButtonPressed(kIntakeToggle);
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
