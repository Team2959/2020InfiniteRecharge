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
    
    frc::SmartDashboard::PutNumber(kName + ": P Gain", m_PID.GetP());
    frc::SmartDashboard::PutNumber(kName + ": I Gain", m_PID.GetI());
    frc::SmartDashboard::PutNumber(kName + ": Feed Forward", m_PID.GetFF());
    frc::SmartDashboard::PutNumber(kName + ": I Zone", m_PID.GetIZone());
}

void Shooter::OnRobotPeriodic()
{    
    // Get the values only once to optimize for speed
    auto currentP = m_PID.GetP();
    auto currentI = m_PID.GetI();
    auto currentFF = m_PID.GetFF();
    auto currentIZone = m_PID.GetIZone();

    auto myP = frc::SmartDashboard::GetNumber(kName + ": P Gain", currentP);
    auto myI = frc::SmartDashboard::GetNumber(kName + ": I Gain", currentI);
    auto myFF = frc::SmartDashboard::GetNumber(kName + ": Feed Forward", currentFF);
    auto myIZone = frc::SmartDashboard::GetNumber(kName + ": I Zone", currentIZone);
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

void Shooter::SetKickerSpeed(double speed)
{
    m_kickerMotor.Set(speed);
}
