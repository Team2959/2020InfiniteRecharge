#include <subsystems/Intake.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Intake::OnRobotInit()
{
    // Debug Enable
    frc::SmartDashboard::PutBoolean(kDebug, m_debugEnable);
    // Intake
    frc::SmartDashboard::PutNumber(kIntakeSpeed, kFullIntakeSpeed);
    // Conveyor
    frc::SmartDashboard::PutNumber(kConveyorSpeed, kFullConveyorSpeed);
    // Kicker
    frc::SmartDashboard::PutNumber(kKickerSpeed, kFullKickerSpeed);
}

void Intake::OnRobotPeriodic()
{
    m_debugEnable = frc::SmartDashboard::GetBoolean(kDebug, false);

    if (m_debugEnable == false) return;
    m_intakeSpeed = frc::SmartDashboard::GetNumber(kIntakeSpeed, kFullIntakeSpeed);
    m_conveyorSpeed = frc::SmartDashboard::GetNumber(kConveyorSpeed, kFullConveyorSpeed);
    m_kickerSpeed = frc::SmartDashboard::GetNumber(kKickerSpeed, kFullKickerSpeed);
}

void Intake::ProcessStickySwitches()
{
    m_stopKickerSensor.ProcessForPressed();
    m_startKickerSensor.ProcessForPressed();
    m_newPowercellSensor.ProcessForPressed();
    m_securedPowercellSensor.ProcessForPressed();
}

bool Intake::GetSensor(Intake::SensorLocation location)
{
    switch(location)
    {
    case Intake::SensorLocation::StartKicker:
        return m_startKickerSensor.Get();
    case Intake::SensorLocation::StopKicker:
        return m_stopKickerSensor.Get();
    case Intake::SensorLocation::NewPowercell:
        return m_newPowercellSensor.Get();
    case Intake::SensorLocation::SecuredPowercell:
        return m_securedPowercellSensor.Get();
    }
    return false;
}

bool Intake::GetSensorPressed(Intake::SensorLocation location)
{
    switch(location)
    {
    case Intake::SensorLocation::StartKicker:
        return m_startKickerSensor.GetPressed();
    case Intake::SensorLocation::StopKicker:
        return m_stopKickerSensor.GetPressed();
    case Intake::SensorLocation::NewPowercell:
        return m_newPowercellSensor.GetPressed();
    case Intake::SensorLocation::SecuredPowercell:
        return m_securedPowercellSensor.GetPressed();
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

double Intake::GetIntakeFullSpeed() const
{
    return m_intakeSpeed;
}

double Intake::GetConveyorFullSpeed() const
{
    return m_conveyorSpeed;
}

double Intake::GetKickerFullSpeed() const
{
    return m_kickerSpeed;
}

bool Intake::IsIntakeRunning() const
{
    return m_intakePrimary.Get() != 0.0;
}
