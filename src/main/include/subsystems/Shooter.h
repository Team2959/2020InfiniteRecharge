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

    frc::Solenoid m_angleAdjuster {kShooterPnuematicsAngleAdjuster};

    // Smart Dashboard
    const std::string kName = "Shooter: ";
    const std::string kDebug = kName + "Debug";
    const std::string kPGain = kName + "P Gain";
    const std::string kIGain = kName + "I Gain";
    const std::string kFF = kName + "Feed Forward";
    const std::string kIZone = kName + "I Zone";
    const std::string kSpeed = kName + "Speed";
    const std::string kTargetSpeed = kName + "Target Speed";
    const std::string kAngle = kName + "Angle";
    const std::string kCloseSpeed = kName + "Close Speed";
    const std::string kAppliedOutput = kName + "Applied Output";

    const double kCloseSpeedDefault = 200;

    double m_closeSpeed = kCloseSpeedDefault;
    double m_targetSpeed = 0;

    bool m_debugEnable;

    void SmartDashboardInit();

public:
    static constexpr double kMaxVelocity = 4500.0;
    static constexpr double kHalfMaxVelocity = 4500.0 / 2.0;

    Shooter();

    void OnRobotInit();
    void OnRobotPeriodic();

    void SetSpeed(double speed);
    double GetSpeed();
    void SetAngle(bool closeShot);
    bool GetAngle();

    bool CloseToSpeed();
};
