#include <subsystems/Intake.h>
#include <frc/smartdashboard/SmartDashboard.h>

Intake::Intake()
{
}

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

    m_intakePrimary.Set(frc::SmartDashboard::GetNumber(kIntakeSpeed, 0));
    m_conveyorPrimary.Set(frc::SmartDashboard::GetNumber(kConveyorSpeed, 0));
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
