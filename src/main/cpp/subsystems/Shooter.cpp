#include <subsystems/Shooter.h>
#include <frc/smartdashboard/SmartDashboard.h>

Shooter::Shooter()
{
    // Have the follower follow the primary except invert 
    // because they are opposite of one another
    m_follower.Follow(m_primary, true);
}

void Shooter::OnRobotInit()
{
    m_PID.SetP(0.0002);
    m_PID.SetFF(0.000193);
    
    SmartDashboardInit();
}

void Shooter::SmartDashboardInit()
{
    // Debug Enable
    frc::SmartDashboard::PutBoolean(kDebug, m_debugEnable);
    // PID
    frc::SmartDashboard::PutNumber(kPGain, m_PID.GetP());
    frc::SmartDashboard::PutNumber(kIGain, m_PID.GetI());
    frc::SmartDashboard::PutNumber(kFF, m_PID.GetFF());
    frc::SmartDashboard::PutNumber(kIZone, m_PID.GetIZone());
    // Shooter
    frc::SmartDashboard::PutNumber(kSpeed, 0);
    frc::SmartDashboard::PutNumber(kTargetSpeed, 0);
    // Angle
    frc::SmartDashboard::PutBoolean(kAngle, false);
}

void Shooter::OnRobotPeriodic()
{
    m_debugEnable = frc::SmartDashboard::GetBoolean(kDebug, false);
    if (m_debugEnable == false) return;

    // Get the values only once to optimize for speed
    auto currentP = m_PID.GetP();
    auto currentI = m_PID.GetI();
    auto currentFF = m_PID.GetFF();
    auto currentIZone = m_PID.GetIZone();

    auto myP = frc::SmartDashboard::GetNumber(kPGain, currentP);
    auto myI = frc::SmartDashboard::GetNumber(kIGain, currentI);
    auto myFF = frc::SmartDashboard::GetNumber(kFF, currentFF);
    auto myIZone = frc::SmartDashboard::GetNumber(kIZone, currentIZone);
    if(fabs(myP - currentP) > kCloseToSameValue)
    {
        m_PID.SetP(myP);
    }
    if(fabs(myI - currentI) > kCloseToSameValue)
    {
        m_PID.SetI(myI);
    }
    if(fabs(myFF - currentFF) > kCloseToSameValue)
    {
        m_PID.SetFF(myFF);
    }
    if(fabs(myIZone - currentIZone) > kCloseToSameValue)
    {
        m_PID.SetIZone(myIZone);
    }

    frc::SmartDashboard::PutNumber(kSpeed, GetSpeed());
}

void Shooter::OnTeleOpPeriodicDebug()
{
    if (m_debugEnable == false) return;

    SetAngle(frc::SmartDashboard::GetBoolean(kAngle, false));
    SetSpeed(frc::SmartDashboard::GetNumber(kTargetSpeed, 0));
}

void Shooter::SetSpeed(double speed)
{
    if (speed >= 0) 
    {
        m_PID.SetReference(speed, rev::ControlType::kVelocity);
    }
}

double Shooter::GetSpeed()
{
    return m_encoder.GetVelocity();
}

void Shooter::SetAngle(bool closeShot)
{
    m_angleAdjuster.Set(closeShot);
}

bool Shooter::GetAngle()
{
    return m_angleAdjuster.Get();
}