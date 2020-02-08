#pragma once

#include <frc/DigitalInput.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>

#include <RobotMap.h>

class Intake
{
private:
    frc::DigitalInput m_endSensor {kEndSensor};
    frc::DigitalInput m_newPowercellSensor {kNewPowercellSensor};
    frc::DigitalInput m_securedPowercellSensor {kSecuredPowercellSensor};

    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_intakePrimary {kIntakeMotor1};
    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_intakeFollower {kIntakeMotor2};

    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_conveyorPrimary {kConveyorMotor1};
    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_conveyorFollower {kConveyorMotor2};

    // Smart Dashboard
    const std::string kDebug = "Intake/Conveyor: Debug";
    const std::string kIntakeName = "Intake: ";
    const std::string kIntakeSpeed = kIntakeName + "Speed";
    const std::string kConveyorName = "Conveyor: ";
    const std::string kConveyorSpeed = kConveyorName + "Speed";

    bool m_debugEnable;

    void SmartDashboardInit();

public:
    enum class SensorLocation
    {
        End,
        NewPowercell,
        SecuredPowercell
    };

    Intake();

    void OnRobotInit();
    void OnRobotPeriodic();
    void OnTeleOpPeriodicDebug();

    bool GetSensor(SensorLocation location);
};