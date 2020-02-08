#pragma once

#include <frc/DigitalInput.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>

#include <RobotMap.h>

class Intake
{
private:
    frc::DigitalInput m_endSensor {kEndSensor};
    frc::DigitalInput m_newPowercellSensor {kNewPowercellSensor};
    frc::DigitalInput m_securedPowercellSensor {kSecuredPowercellSensor};

    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_intakePrimary {kIntakePrimary};
    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_intakeFollower {kIntakeFollower};

    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_conveyorPrimary {kConveyorPrimary};
    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_conveyorFollower {kConveyorFollower};

    ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_kickerMotor {kShooterKicker};

    // Smart Dashboard
    const std::string kDebug = "Intake/Conveyor: Debug";
    const std::string kIntakeName = "Intake: ";
    const std::string kIntakeSpeed = kIntakeName + "Speed";
    const std::string kConveyorName = "Conveyor: ";
    const std::string kConveyorSpeed = kConveyorName + "Speed";
    const std::string kKickerSpeed = kConveyorName + "Kicker Speed";

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

    void SetIntakeSpeed(double speed);
    void SetConveyorSpeed(double speed);
    void SetKickerSpeed(double speed);
    bool GetSensor(SensorLocation location);
};