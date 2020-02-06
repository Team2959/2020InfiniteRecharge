#pragma once

#include <RobotMap.h>
#include <rev/CANSparkMax.h>
#include <frc/Solenoid.h>

#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>

class Shooter
{
private:
    rev::CANSparkMax m_primary {kShooterPrimary, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax m_follower {kShooterFollower, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANEncoder m_encoder {m_primary};
    rev::CANPIDController m_PID {m_primary};

    frc::Solenoid m_angleAdjuster{kShooterPnuematicsAngleAdjuster};

    std::string kName = "Shooter";

    ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_kickerMotor{kShooterKicker};

public:
    Shooter();

    void OnRobotInit();
    void OnRobotPeriodic();

    void SetSpeed(double speed);
    double GetSpeed();
    void SetAngle(bool on);
    void SetKickerSpeed(double speed);
};
