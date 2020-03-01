#pragma once

#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <RobotMap.h>

class Climb
{
private:
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_left {kClimbLeftTalonSrxCanId};
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_right {kClimbRightTalonSrxCanId};

    ctre::phoenix::motorcontrol::can::SlotConfiguration m_pidConfig;
    ctre::phoenix::motorcontrol::can::SlotConfiguration m_pidConfigRight;

    const double kDefaultKp = 0.001;
    const double kDefaultKi = 0;
    const double kDefaultFf = 0;
    const double kDefaultIzone = 0;
    const double kDefaultCruiseVelocity = 5000;
    const double kDefaultAcceleration = 4500;
    
    // Smart Dashboard
    const std::string kName = "Climb/";
    const std::string kDebug = kName + "Debug";
    const std::string kPGain = kName + "P Gain";
    const std::string kIGain = kName + "I Gain";
    const std::string kFF = kName + "Feed Forward";
    const std::string kIZone = kName + "I Zone";
    const std::string kCruiseVelocity = kName + "Cruise Velocity";
    const std::string kAcceleration = kName + "Acceleralation";
    const std::string kPosition = kName + "Position";
    const std::string kVelocity = kName + "Velocity";
    const std::string kTargetPosition = kName + "Target Position";
    const std::string kGoToPosition = kName + "Go To Position";
    const std::string kResetEncoders = kName + "Reset Encoders";

    bool m_debugEnable = false;
    double m_cruiseVelocity = kDefaultCruiseVelocity;
    double m_acceleration = kDefaultAcceleration;

    double m_lastGoToPosition = 0.0;

    void MoveToPosition(double target);
    void StopAndZero();

public:
    void OnRobotInit();
    void OnRobotPeriodic();
};
