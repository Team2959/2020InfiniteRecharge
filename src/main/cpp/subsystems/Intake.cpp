#include <subsystems/Intake.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Intake::OnRobotInit()
{
    m_intakeFollower.Follow(m_intakePrimary);
    m_conveyorFollower.Follow(m_conveyorPrimary);

    SmartDashboardInit();
}

void Intake::SmartDashboardInit()
{
    // Debug Enable
    frc::SmartDashboard::PutBoolean(kDebug, m_debugEnable);
    // Intake/Conveyor
    frc::SmartDashboard::PutNumber(kIntakeSpeed, 0);
    frc::SmartDashboard::PutNumber(kConveyorSpeed, 0);
}

void Intake::OnRobotPeriodic()
{
    m_debugEnable = frc::SmartDashboard::GetBoolean(kDebug, false);
}

void Intake::OnTeleOpPeriodicDebug()
{
    if (m_debugEnable == false) return;

    auto currentIntakeSpeed = m_intakePrimary.Get();
    auto myIntakeSpeed = frc::SmartDashboard::GetNumber(kIntakeSpeed, currentIntakeSpeed);
    if(fabs(myIntakeSpeed - currentIntakeSpeed) > kCloseToSameValue)
    {
        m_intakePrimary.Set(myIntakeSpeed);
    }

    auto currentConveyorSpeed = m_conveyorPrimary.Get();
    auto myConveyorSpeed = frc::SmartDashboard::GetNumber(kConveyorSpeed, currentConveyorSpeed);
    if(fabs(myConveyorSpeed - currentConveyorSpeed) > kCloseToSameValue)
    {
        m_conveyorPrimary.Set(myConveyorSpeed);
    }
}

bool Intake::GetSensor(Intake::SensorLocation location)
{
    switch(location)
    {
    case Intake::SensorLocation::End:
        return m_endSensor.Get();
    case Intake::SensorLocation::NewPowercell:
        return m_newPowercellSensor.Get();
    case Intake::SensorLocation::SecuredPowercell:
        return m_securedPowercellSensor.Get();
    }
    return false;
}
