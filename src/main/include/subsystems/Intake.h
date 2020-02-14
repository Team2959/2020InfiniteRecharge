#pragma once

#include <frc/DigitalInput.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>

#include <RobotMap.h>

class Intake
{
private:
    frc::DigitalInput m_startKickerSensor {kStartKickerSensor}; 
    frc::DigitalInput m_stopKickerSensor {kStopKickerSensor};
    frc::DigitalInput m_newPowercellSensor {kNewPowercellSensor};
    frc::DigitalInput m_securedPowercellSensor {kSecuredPowercellSensor};

    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_intakePrimary {kIntakePrimary};
    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_kickerMotor {kKicker};
    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_conveyorPrimary {kConveyorPrimary};
    // ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_conveyorFollower {kConveyorFollower};
    
    // Smart Dashboard
    const std::string kDebug = "Intake/Conveyor: Debug";
    const std::string kIntakeName = "Intake: ";
    const std::string kConveyorName = "Conveyor: ";
    const std::string kConveyorSpeed = kConveyorName + "Speed";
    const std::string kKickerSpeed = kConveyorName + "Kicker Speed";

    bool m_debugEnable;

    void SmartDashboardInit();

public:
    // temporary  moving of this value to the public for debug
    const std::string kIntakeSpeed = kIntakeName + "Speed";
    enum class SensorLocation
    {
        StartKicker,
        StopKicker,
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
    bool GetSensorPressed(SensorLocation location);
};