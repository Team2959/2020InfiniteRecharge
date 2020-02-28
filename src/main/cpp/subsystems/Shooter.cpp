#include <subsystems/Shooter.h>
#include <frc/smartdashboard/SmartDashboard.h>

Shooter::Shooter()
{
    // Have the follower follow the primary except invert 
    // because they are opposite of one another
    m_follower.Follow(m_primary, true);

    ComputeSlopeAndOffset();
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
    // Close Speed
    frc::SmartDashboard::PutNumber(kCloseSpeed, kCloseSpeedDefault);
    // Applied Output
    frc::SmartDashboard::PutNumber(kAppliedOutput, m_primary.GetAppliedOutput());
    // Min and Max Throttle Speeds
    frc::SmartDashboard::PutNumber(kMaxThrottleSpeed, kMaxThrottleSpeedDefault);
    frc::SmartDashboard::PutNumber(kMinThrottleSpeed, kMinThrottleSpeedDefault);
}

void Shooter::OnRobotPeriodic()
{
    frc::SmartDashboard::PutNumber(kSpeed, -GetSpeed());
    frc::SmartDashboard::PutNumber(kAppliedOutput, m_primary.GetAppliedOutput());
    frc::SmartDashboard::PutString(kAngle, GetHoodSwitchStateText());

    m_debugEnable = frc::SmartDashboard::GetBoolean(kDebug, false);
    if (m_debugEnable == false) return;

    m_maxThrottleRange = frc::SmartDashboard::GetNumber(kMaxThrottleSpeed, kMaxThrottleSpeedDefault);
    m_minThrottleRange = frc::SmartDashboard::GetNumber(kMinThrottleSpeed, kMinThrottleSpeedDefault);
    ComputeSlopeAndOffset();

    // Close Speed
    m_closeSpeed = frc::SmartDashboard::GetNumber(kCloseSpeed, kCloseSpeedDefault);

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
}

std::string Shooter::GetHoodSwitchStateText()
{
    if(GetAngle())
        return "Close";
    else
        return "Far";    
}

double Shooter::GetSpeed()
{
    return m_encoder.GetVelocity();
}

bool Shooter::CloseToSpeed()
{
    return std::fabs(GetSpeed() - m_targetSpeed) <= m_closeSpeed;
}

void Shooter::ComputeSlopeAndOffset()
{
    m_slopeOfThrottleRange = (m_maxThrottleRange - m_minThrottleRange) / (2 - 0.5);
    m_offsetOfThrottleRange = m_minThrottleRange - (m_slopeOfThrottleRange * 0.5);
}

void Shooter::SetSpeedFromThrottle(double throttlePosition)
{
    throttlePosition += 1;  // shift from -1..1 to 0..2 for range
    // 0..0.5, set shooter speed to 0 
    auto targetSpeed = 0.0;
    if (throttlePosition >= 0.5)
    {
        // do linear interpolation between minimum and maximum throttle speeds
        targetSpeed = (m_slopeOfThrottleRange * throttlePosition) + m_offsetOfThrottleRange; 
    }
    SetSpeed(targetSpeed);
}

void Shooter::SetSpeed(double speed)
{
    speed = std::fmax(speed, 0);
    frc::SmartDashboard::PutNumber("Throttle Target Speed", speed);
    // invert speed for primary motor direction
    m_targetSpeed = -1.0 * std::fmin(speed, kMaxVelocity);
    m_PID.SetReference(m_targetSpeed, rev::ControlType::kVelocity);
}

void Shooter::SetAngle(bool closeShot)
{
    m_angleAdjuster.Set(closeShot);
}

bool Shooter::GetAngle()
{
    return m_angleAdjuster.Get();
}
