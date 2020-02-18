/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <subsystems/Drivetrain.h>
#include <frc/smartdashboard/SmartDashboard.h>

Drivetrain::Drivetrain() 
{
    m_leftFollower1.Follow(m_leftPrimary);
    m_leftFollower2.Follow(m_leftPrimary);
    m_rightFollower1.Follow(m_rightPrimary);
    m_rightFollower2.Follow(m_rightPrimary);
}

void Drivetrain::SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds) 
{
    // Don't know what to do here

    // m_leftPID.SetReference(static_cast<double>(speeds.left), rev::ControlType::kVelocity);
    // m_rightPID.SetReference(static_cast<double>(speeds.right), rev::ControlType::kVelocity);
}

void Drivetrain::SetSpeeds(double left, double right)
{
    m_leftPID.SetReference(-left, rev::ControlType::kVelocity);
    m_rightPID.SetReference(right, rev::ControlType::kVelocity);
}

void Drivetrain::InitalShowToSmartDashboard()
{
    m_leftPID.SetFF(0.0005);
    m_rightPID.SetFF(0.0005);
    // Debug Enable
    frc::SmartDashboard::PutBoolean(kDebug, m_debugEnable);
    // PID
    frc::SmartDashboard::PutNumber(kPGain, m_leftPID.GetP());
    frc::SmartDashboard::PutNumber(kIGain, m_leftPID.GetI());
    frc::SmartDashboard::PutNumber(kFF, m_leftPID.GetFF());
    frc::SmartDashboard::PutNumber(kIZone, m_leftPID.GetIZone());
}

void Drivetrain::UpdateFromSmartDashboard()
{    
    m_debugEnable = frc::SmartDashboard::GetBoolean(kDebug, false);
    if (m_debugEnable == false) return;

    // Get the values only once to optimize for speed
    auto currentP = m_leftPID.GetP();
    auto currentI = m_leftPID.GetI();
    auto currentFF = m_leftPID.GetFF();
    auto currentIZone = m_leftPID.GetIZone();

    auto myP = frc::SmartDashboard::GetNumber(kPGain, currentP);
    auto myI = frc::SmartDashboard::GetNumber(kIGain, currentI);
    auto myFF = frc::SmartDashboard::GetNumber(kFF, currentFF);
    auto myIZone = frc::SmartDashboard::GetNumber(kIZone, currentIZone);
    if(fabs(myP - currentP) > kCloseToSameValue)
    {
        m_rightPID.SetP(myP);
        m_leftPID.SetP(myP);
    }
    if(fabs(myI - currentI) > kCloseToSameValue)
    {
        m_rightPID.SetI(myI);
        m_leftPID.SetI(myI);
    }
    if(fabs(myFF - currentFF) > kCloseToSameValue)
    {
        m_rightPID.SetFF(myFF);
        m_leftPID.SetFF(myFF);
    }
    if(fabs(myIZone - currentIZone) > kCloseToSameValue)
    {
        m_rightPID.SetIZone(myIZone);
        m_leftPID.SetIZone(myIZone);
    }
}
