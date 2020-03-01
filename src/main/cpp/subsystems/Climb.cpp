#include <subsystems/Climb.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Climb::OnRobotInit()
{
    m_left.GetSlotConfigs(m_pidConfig);
    m_pidConfig.kP = kDefaultKp;
    m_pidConfig.kI = kDefaultKi;
    m_pidConfig.kF = kDefaultFf;
    m_pidConfig.integralZone = kDefaultIzone;
    m_left.ConfigMotionCruiseVelocity(5000, 10);
    m_left.ConfigMotionAcceleration(4500,10);

    m_right.Follow(m_left);
    m_right.SetInverted( ctre::phoenix::motorcontrol::InvertType::OpposeMaster);

    // Debug Enable
    frc::SmartDashboard::PutBoolean(kDebug, m_debugEnable);
    // PID
    frc::SmartDashboard::PutNumber(kPGain, kDefaultKp);
    frc::SmartDashboard::PutNumber(kIGain, kDefaultKi);
    frc::SmartDashboard::PutNumber(kFF, kDefaultFf);
    frc::SmartDashboard::PutNumber(kIZone, kDefaultIzone);
    frc::SmartDashboard::PutNumber(kCruiseVelocity, kDefaultCruiseVelocity);
    frc::SmartDashboard::PutNumber(kAcceleration, kDefaultAcceleration);
}

void Climb::OnRobotPeriodic()
{
    m_debugEnable = frc::SmartDashboard::GetBoolean(kDebug, false);

    if (m_debugEnable == false) return;

    // Get the values only once to optimize for speed
    auto currentP = m_pidConfig.kP;
    auto currentI = m_pidConfig.kI;
    auto currentFF = m_pidConfig.kF;
    auto currentIZone = m_pidConfig.integralZone;

    auto myP = frc::SmartDashboard::GetNumber(kPGain, currentP);
    auto myI = frc::SmartDashboard::GetNumber(kIGain, currentI);
    auto myFF = frc::SmartDashboard::GetNumber(kFF, currentFF);
    auto myIZone = frc::SmartDashboard::GetNumber(kIZone, currentIZone);
    auto myCruiseV = frc::SmartDashboard::GetNumber(kCruiseVelocity, kDefaultCruiseVelocity);
    auto myAccel = frc::SmartDashboard::GetNumber(kAcceleration, kDefaultAcceleration);
    if(fabs(myP - currentP) > kCloseToSameValue)
    {
        m_pidConfig.kP = myP;
    }
    if(fabs(myI - currentI) > kCloseToSameValue)
    {
        m_pidConfig.kI = myI;
    }
    if(fabs(myFF - currentFF) > kCloseToSameValue)
    {
        m_pidConfig.kF = myFF;
    }
    if(fabs(myIZone - currentIZone) > kCloseToSameValue)
    {
        m_pidConfig.integralZone = myIZone;
    }
    if(fabs(myCruiseV - m_cruiseVelocity) > kCloseToSameValue)
    {
        m_cruiseVelocity = myCruiseV;
        m_left.ConfigMotionCruiseVelocity(m_cruiseVelocity, 10);
    }
    if(fabs(myAccel - currentIZone) > kCloseToSameValue)
    {
        m_acceleration = myAccel;
        m_left.ConfigMotionAcceleration(m_acceleration,10);
    }
}
