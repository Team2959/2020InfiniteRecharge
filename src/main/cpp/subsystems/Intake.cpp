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
    frc::SmartDashboard::PutNumber(kConveyorSpeedWhenLoading, kFullConveyorSpeedWhenLoading);
    // Kicker
    frc::SmartDashboard::PutNumber(kKickerSpeed, kFullKickerSpeed);
    // frc::SmartDashboard::PutNumber(kKickerPulseCycles, kDefaultKickerPulseCycles);
    // frc::SmartDashboard::PutNumber(kKickerPauseCycles, kDefaultKickerPauseCycles);
}

void Intake::OnRobotPeriodic()
{
    m_debugEnable = frc::SmartDashboard::GetBoolean(kDebug, false);
    frc::SmartDashboard::PutBoolean("New Power Cell", GetSensor(Intake::SensorLocation::NewPowercell));
    frc::SmartDashboard::PutBoolean("Secured Power Cell", GetSensor(Intake::SensorLocation::SecuredPowercell));
    frc::SmartDashboard::PutBoolean("Kicker Sensor", GetSensor(Intake::SensorLocation::Kicker));
    frc::SmartDashboard::PutString(kIntakeState, GetIntakeStateText());

    if (m_debugEnable == false) return;
    m_intakeSpeed = frc::SmartDashboard::GetNumber(kIntakeSpeed, kFullIntakeSpeed);
    m_conveyorSpeed = frc::SmartDashboard::GetNumber(kConveyorSpeed, kFullConveyorSpeed);
    m_conveyorSpeedWhenLoading = frc::SmartDashboard::GetNumber(kConveyorSpeedWhenLoading, kFullConveyorSpeedWhenLoading);
    m_kickerSpeed = frc::SmartDashboard::GetNumber(kKickerSpeed, kFullKickerSpeed);
    // m_PulseCycles = static_cast<int>(frc::SmartDashboard::GetNumber(kKickerPulseCycles, kDefaultKickerPulseCycles));
    // m_PauseCycles = static_cast<int>(frc::SmartDashboard::GetNumber(kKickerPauseCycles, kDefaultKickerPauseCycles));
}

std::string Intake::GetIntakeStateText()
{
    if(IsIntakeRunning())
        return "On";
    else
        return "Off";
}

void Intake::ProcessStickySwitches()
{
    m_kickerSensor.ProcessForPressed();
    m_newPowercellSensor.ProcessForPressed();
    m_securedPowercellSensor.ProcessForPressed();
}

bool Intake::GetSensor(Intake::SensorLocation location)
{
    switch(location)
    {
    case Intake::SensorLocation::Kicker:
        return m_kickerSensor.Get();
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
    case Intake::SensorLocation::Kicker:
        return m_kickerSensor.GetPressed();
    case Intake::SensorLocation::NewPowercell:
        return m_newPowercellSensor.GetPressed();
    case Intake::SensorLocation::SecuredPowercell:
        return m_securedPowercellSensor.GetPressed();
    }
    return false;
}

bool Intake::IsIntakeRunning() const
{
    return m_intake.Get() != 0.0;
}

double Intake::GetIntakeFullSpeed() const
{
    return m_intakeSpeed;
}

void Intake::SetIntakeSpeed(double speed)
{
    m_intake.Set(speed);
}

double Intake::GetConveyorFullSpeed() const
{
    return m_conveyorSpeed;
}

double Intake::GetConveyorFullSpeedWhenLoading() const
{
    return m_conveyorSpeedWhenLoading;
}

void Intake::SetConveyorSpeed(double speed)
{
    m_conveyor.Set(speed);
}

double Intake::GetKickerFullSpeed() const
{
    return m_kickerSpeed;
}

void Intake::SetKickerSpeed(double speed)
{
    m_kicker.Set(-speed);
}

// int Intake::GetKickerPulseCycles() const
// {
//     return m_PulseCycles;
// }

// int Intake::GetKickerPauseCycles() const
// {
//     return m_PauseCycles;
// }
