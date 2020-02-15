#pragma once

#include <frc/DigitalInput.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>

#include <RobotMap.h>

#include <utility/StickySwitch.h>

class Intake
{
private:
    cwtech::StickySwitch m_startKickerSensor {kStartKickerSensor}; 
    cwtech::StickySwitch m_stopKickerSensor {kStopKickerSensor};
    cwtech::StickySwitch m_newPowercellSensor {kNewPowercellSensor};
    cwtech::StickySwitch m_securedPowercellSensor {kSecuredPowercellSensor};

    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_intakePrimary {kIntakePrimary};
    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_kickerMotor {kKicker};
    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_conveyorPrimary {kConveyorPrimary};
    
    // Smart Dashboard
    const std::string kDebug = "Intake/Conveyor: Debug";
    const std::string kIntakeName = "Intake: ";
    const std::string kConveyorName = "Conveyor: ";
    const std::string kConveyorSpeed = kConveyorName + "Speed";
    const std::string kKickerSpeed = kConveyorName + "Kicker Speed";
    const std::string kIntakeSpeed = kIntakeName + "Speed";

    bool m_debugEnable = false;

    const double kFullIntakeSpeed = 0.5;
    const double kFullConveyorSpeed = 0.9;
    const double kFullKickerSpeed = 0.9;
    double m_intakeSpeed = kFullIntakeSpeed;
    double m_conveyorSpeed = kFullConveyorSpeed;
    double m_kickerSpeed = kFullKickerSpeed;

public:
    enum class SensorLocation
    {
        StartKicker,
        StopKicker,
        NewPowercell,
        SecuredPowercell
    };

    void OnRobotInit();
    void OnRobotPeriodic();

    double GetIntakeFullSpeed() const;
    double GetConveyorFullSpeed() const;
    double GetKickerFullSpeed() const;
    bool IsIntakeRunning() const;

    void SetIntakeSpeed(double speed);
    void SetConveyorSpeed(double speed);
    void SetKickerSpeed(double speed);

    bool GetSensor(SensorLocation location);
    bool GetSensorPressed(SensorLocation location);
};