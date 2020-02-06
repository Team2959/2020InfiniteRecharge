#pragma once

#include <RobotMap.h>
#include <rev/CANSparkMax.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <frc/Solenoid.h>

class Shooter
{
private:
    std::string kName = "Shooter";

    rev::CANSparkMax m_primary {kShooterPrimary, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax m_follower {kShooterFollower, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANEncoder m_encoder {m_primary};
    rev::CANPIDController m_PID {m_primary};

    ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_kickerMotor {kShooterKicker};

    frc::Solenoid m_angleAdjuster {kShooterPnuematicsAngleAdjuster};

public:
    Shooter();

    void OnRobotInit();
    void OnRobotPeriodic();

    void SetSpeed(double speed);
    double GetSpeed();
    void SetAngle(bool closeShot);
    void SetKickerSpeed(double speed);
};
