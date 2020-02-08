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
    // Intake
    frc::SmartDashboard::PutNumber(kIntakeSpeed, 0);
    // Conveyor
    frc::SmartDashboard::PutNumber(kConveyorSpeed, 0);
    // Kicker
    frc::SmartDashboard::PutNumber(kKickerSpeed, 0);
}

void Intake::OnRobotPeriodic()
{
    m_debugEnable = frc::SmartDashboard::GetBoolean(kDebug, false);
}

void Intake::OnTeleOpPeriodicDebug()
{
    if (m_debugEnable == false) return;

    SetIntakeSpeed(frc::SmartDashboard::GetNumber(kIntakeSpeed, 0));
    SetConveyorSpeed(frc::SmartDashboard::GetNumber(kConveyorSpeed, 0));
    SetKickerSpeed(frc::SmartDashboard::GetNumber(kKickerSpeed, 0));
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

void Intake::SetIntakeSpeed(double speed)
{
    m_intakePrimary.Set(speed);
}

void Intake::SetConveyorSpeed(double speed)
{
    m_conveyorPrimary.Set(speed);
}

void Intake::SetKickerSpeed(double speed)
{
    m_kickerMotor.Set(speed);
}
