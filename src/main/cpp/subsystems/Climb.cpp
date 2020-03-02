#include <subsystems/Climb.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Climb::OnRobotInit()
{
    m_left.Config_kP(0, m_kP);
    m_left.Config_kI(0, m_kI);
    m_left.Config_kF(0, m_kFF);
    m_left.Config_IntegralZone(0, m_kIZone);
    m_right.Config_kP(0, m_kP);
    m_right.Config_kI(0, m_kI);
    m_right.Config_kF(0, m_kFF);
    m_right.Config_IntegralZone(0, m_kIZone);
    
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

    auto myP = frc::SmartDashboard::GetNumber(kPGain, m_kP);
    auto myI = frc::SmartDashboard::GetNumber(kIGain, m_kI);
    auto myFF = frc::SmartDashboard::GetNumber(kFF, m_kFF);
    auto myIZone = frc::SmartDashboard::GetNumber(kIZone, m_kIZone);
    auto myCruiseV = frc::SmartDashboard::GetNumber(kCruiseVelocity, kDefaultCruiseVelocity);
    auto myAccel = frc::SmartDashboard::GetNumber(kAcceleration, kDefaultAcceleration);
    if(fabs(myP - m_kP) > kCloseToSameValue)
    {
        m_kP = myP;
        m_left.Config_kP(0, m_kP);
        m_right.Config_kP(0, m_kP);
    }
    if(fabs(myI - m_kI) > kCloseToSameValue)
    {
        m_kI = myI;
        m_left.Config_kI(0, m_kI);
        m_right.Config_kI(0, m_kI);
    }
    if(fabs(myFF - m_kFF) > kCloseToSameValue)
    {
        m_kFF = myFF;
        m_left.Config_kF(0, m_kFF);
        m_right.Config_kF(0, m_kFF);
    }
    if(fabs(myIZone - m_kIZone) > kCloseToSameValue)
    {
        m_kIZone = myIZone;
        m_left.Config_IntegralZone(0, m_kIZone);
        m_right.Config_IntegralZone(0, m_kIZone);
    }
    if(fabs(myCruiseV - m_cruiseVelocity) > kCloseToSameValue)
    {
        m_cruiseVelocity = myCruiseV;
        m_left.ConfigMotionCruiseVelocity(m_cruiseVelocity, 10);
        m_right.ConfigMotionCruiseVelocity(m_cruiseVelocity, 10);
    }
    if(fabs(myAccel - m_acceleration) > kCloseToSameValue)
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