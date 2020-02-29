#pragma once

#include <RobotMap.h>
#include <rev/CANSparkMax.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <frc/Solenoid.h>

class Shooter
{
private:
    // Hardware
    rev::CANSparkMax m_primary {kShooterPrimary, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax m_follower {kShooterFollower, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANEncoder m_encoder {m_primary};
    rev::CANPIDController m_PID {m_primary};

    frc::Solenoid m_angleAdjuster {kShooterAngleAdjusterPcmId};

    // Smart Dashboard
    const std::string kName = "Shooter/";
    const std::string kDebug = kName + "Debug";
    const std::string kPGain = kName + "P Gain";
    const std::string kIGain = kName + "I Gain";
    const std::string kFF = kName + "Feed Forward";
    const std::string kIZone = kName + "I Zone";
    const std::string kSpeed = kName + "Speed";
    const std::string kTargetSpeed = kName + "Target Speed";
    const std::string kAngle = kName + "Hood Angle";
    const std::string kCloseSpeed = kName + "Close Speed";
    const std::string kAppliedOutput = kName + "Applied Output";
    const std::string kMaxThrottleSpeed = kName + "Max Throttle Speed";
    const std::string kMinThrottleSpeed = kName + "Min Throttle Speed";

    const double kMaxVelocity = 4500;
    const double kMaxThrottleSpeedDefault = 4000;
    const double kMinThrottleSpeedDefault = 1500;
    const double kCloseSpeedDefault = 200;

    double m_closeSpeed = kCloseSpeedDefault;
    double m_targetSpeed = 0;
    double m_maxThrottleRange = kMaxThrottleSpeedDefault;
    double m_minThrottleRange = kMinThrottleSpeedDefault;
    double m_slopeOfThrottleRange = 1;
    double m_offsetOfThrottleRange = 0;

    bool m_debugEnable;

    void SmartDashboardInit();
    void ComputeSlopeAndOffset();
    std::string GetHoodSwitchStateText();

    double GetSpeed();
    void SetSpeed(double speed);

public:
    Shooter();

    void OnRobotInit();
    void OnRobotPeriodic();

    bool CloseToSpeed();
    void SetSpeedFromThrottle(double throttlePositon);

    void SetAngle(bool closeShot);
    bool GetAngle();
};
