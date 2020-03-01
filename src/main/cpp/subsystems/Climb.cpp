#include <subsystems/Climb.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Climb::OnRobotInit()
{
    m_left.GetSlotConfigs(m_pidConfig);
    m_right.GetSlotConfigs(m_pidConfigRight);

    m_pidConfig.kP = kDefaultKp;
    m_pidConfig.kI = kDefaultKi;
    m_pidConfig.kF = kDefaultFf;
    m_pidConfig.integralZone = kDefaultIzone;
    m_pidConfigRight.kP = kDefaultKp;
    m_pidConfigRight.kI = kDefaultKi;
    m_pidConfigRight.kF = kDefaultFf;
    m_pidConfigRight.integralZone = kDefaultIzone;
    
    m_left.ConfigMotionCruiseVelocity(kDefaultCruiseVelocity, 10);
    m_left.ConfigMotionAcceleration(kDefaultAcceleration, 10);
    m_right.ConfigMotionCruiseVelocity(kDefaultCruiseVelocity, 10);
    m_right.ConfigMotionAcceleration(kDefaultAcceleration, 10);

    // Debug Enable
    frc::SmartDashboard::PutBoolean(kDebug, m_debugEnable);
    // PID
    frc::SmartDashboard::PutNumber(kPGain, kDefaultKp);
    frc::SmartDashboard::PutNumber(kIGain, kDefaultKi);
    frc::SmartDashboard::PutNumber(kFF, kDefaultFf);
    frc::SmartDashboard::PutNumber(kIZone, kDefaultIzone);
    // Magic motion
    frc::SmartDashboard::PutNumber(kCruiseVelocity, kDefaultCruiseVelocity);
    frc::SmartDashboard::PutNumber(kAcceleration, kDefaultAcceleration);
    frc::SmartDashboard::PutNumber(kPosition, 0);
    frc::SmartDashboard::PutNumber(kRightPosition, 0);
    frc::SmartDashboard::PutNumber(kVelocity, 0);
    frc::SmartDashboard::PutNumber(kTargetPosition, 0);
    frc::SmartDashboard::PutNumber(kGoToPosition, 0);
    frc::SmartDashboard::PutBoolean(kResetEncoders, false);
 
    StopAndZero();
}

void Climb::OnRobotPeriodic()
{
    m_debugEnable = frc::SmartDashboard::GetBoolean(kDebug, false);

    frc::SmartDashboard::PutNumber(kPosition, m_left.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber(kRightPosition, m_right.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber(kVelocity, m_left.GetSelectedSensorVelocity());

    if (m_debugEnable == false) return;

    if (frc::SmartDashboard::GetBoolean(kResetEncoders, false))
    {
        StopAndZero();
        frc::SmartDashboard::PutBoolean(kResetEncoders, true);
    }

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
        m_pidConfigRight.kP = myP;
    }
    if(fabs(myI - currentI) > kCloseToSameValue)
    {
        m_pidConfig.kI = myI;
        m_pidConfigRight.kI = myI;
    }
    if(fabs(myFF - currentFF) > kCloseToSameValue)
    {
        m_pidConfig.kF = myFF;
        m_pidConfigRight.kF = myFF;
    }
    if(fabs(myIZone - currentIZone) > kCloseToSameValue)
    {
        m_pidConfig.integralZone = myIZone;
        m_pidConfigRight.integralZone = myIZone;
    }
    if(fabs(myCruiseV - m_cruiseVelocity) > kCloseToSameValue)
    {
        m_cruiseVelocity = myCruiseV;
        m_left.ConfigMotionCruiseVelocity(m_cruiseVelocity, 10);
        m_right.ConfigMotionCruiseVelocity(m_cruiseVelocity, 10);
    }
    if(fabs(myAccel - currentIZone) > kCloseToSameValue)
    {
        m_acceleration = myAccel;
        m_left.ConfigMotionAcceleration(m_acceleration,10);
        m_right.ConfigMotionAcceleration(m_acceleration,10);
    }

    double position = frc::SmartDashboard::GetNumber(kGoToPosition, 0.0);
    if(std::fabs(position - m_lastGoToPosition) > kCloseToSameValue)
    {
        MoveToPosition(position);
        m_lastGoToPosition = position;
    }
}

void Climb::MoveToPosition(double target)
{
    m_left.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, target);
    m_right.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, -target);
    frc::SmartDashboard::PutNumber(kTargetPosition, target);
}

void Climb::StopAndZero()
{
    m_left.StopMotor();
    m_right.StopMotor();
    m_left.SetSelectedSensorPosition(0,0,0);
    m_right.SetSelectedSensorPosition(0,0,0);
}