#pragma once

#include <frc/DigitalInput.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>

#include <RobotMap.h>

#include <utility/StickySwitch.h>

class Intake
{
private:
    cwtech::StickySwitch m_newPowercellSensor {kNewPowercellSensor};
    cwtech::StickySwitch m_securedPowercellSensor {kSecuredPowercellSensor};
    cwtech::StickySwitch m_kickerSensor {kKickerSensor};

    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_intakePrimary {kIntakePrimary};
    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_kickerMotor {kKicker};
    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_conveyorPrimary {kConveyorPrimary};
    
    // Smart Dashboard
    const std::string kDebug = "Intake/Conveyor: Debug";
    const std::string kIntakeName = "Intake: ";
    const std::string kConveyorName = "Conveyor: ";
    const std::string kKickerName = "Kicker: ";
    const std::string kConveyorSpeed = kConveyorName + "Speed";
    const std::string kKickerSpeed = kKickerName + "Speed";
    const std::string kIntakeSpeed = kIntakeName + "Speed";
    const std::string kKickerPulseCycles = kKickerName + "Pulse Cycles";
    const std::string kKickerPauseCycles = kKickerName + "Pause Cycles";
    const std::string kConveyorSpeedWhenLoading = kConveyorName + "Speed When Loading";

    bool m_debugEnable = false;

    const double kFullIntakeSpeed = 0.5;
    const double kFullConveyorSpeed = 0.6;
    const double kFullKickerSpeed = 0.3;
    const double kFullConveyorSpeedWhenLoading = 0.9;
    const int kDefaultKickerPulseCycles = 5;
    const int kDefaultKickerPauseCycles = 5;
    double m_intakeSpeed = kFullIntakeSpeed;
    double m_conveyorSpeed = kFullConveyorSpeed;
    double m_conveyorSpeedWhenLoading = kFullConveyorSpeedWhenLoading;
    double m_kickerSpeed = kFullKickerSpeed;
    int m_PulseCycles = kDefaultKickerPulseCycles;
    int m_PauseCycles = kDefaultKickerPauseCycles;

public:
    enum class SensorLocation
    {
        Kicker,
        NewPowercell,
        SecuredPowercell
    };

    void OnRobotInit();
    void OnRobotPeriodic();
    void ProcessStickySwitches();

    double GetIntakeFullSpeed() const;
    double GetConveyorFullSpeed() const;
    double GetConveyorFullSpeedWhenLoading() const;
    double GetKickerFullSpeed() const;
    bool IsIntakeRunning() const;
    int GetKickerPulseCycles() const;
    int GetKickerPauseCycles() const;

    void SetIntakeSpeed(double speed);
    void SetConveyorSpeed(double speed);
    void SetKickerSpeed(double speed);

    bool GetSensor(SensorLocation location);
    bool GetSensorPressed(SensorLocation location);
};